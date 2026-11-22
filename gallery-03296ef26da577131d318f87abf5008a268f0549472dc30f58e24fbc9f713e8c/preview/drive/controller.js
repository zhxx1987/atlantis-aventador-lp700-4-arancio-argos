import { Quaternion, Vector3 } from 'three';

const wheels = ['FrontL', 'FrontR', 'RearL', 'RearR'];
const doorNames = ['Door_L', 'Door_R'];
const keymap = { KeyW: 'w', KeyS: 's', KeyA: 'a', KeyD: 'd', Space: 'space' };
const center = new Vector3(2.44, 0, 0);
const vertical = new Vector3(0, 1, 0);
const axle = new Vector3(0, 0, 1);
const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
const wrapCourse = x => ((x + 216) % 432 + 432) % 432 - 216;

const STEP_MS = 1000 / 120; // motion.py advances exactly 120 fixed steps per second.
const PRESENTATION_DELAY_MS = 65;
const TIMELINE_GAP_MS = 180;
const scalarChannels = ['x', 'z', 'height', 'yaw', 'pitch', 'roll', 'speed'];
const arrayChannels = ['travel', 'wheel_steer', 'spin', 'door'];

export class DrivePresentation {
  samples = [];
  origin = 0;
  cursor = -Infinity;

  reset() {
    this.samples.length = 0;
    this.cursor = -Infinity;
  }

  push(pose, now) {
    const time = pose.frames * STEP_MS;
    const last = this.samples.at(-1);
    if (last && time <= last.time) return;
    if (!last || time - last.time > TIMELINE_GAP_MS || now - this.origin - time > TIMELINE_GAP_MS) {
      // After a long interruption use the new live pose instead of replaying a stale backlog.
      this.reset();
      this.origin = now - time;
    }
    this.samples.push({ time, pose });
    if (this.samples.length > 32) this.samples.shift();
  }

  sample(now) {
    const first = this.samples[0];
    const last = this.samples.at(-1);
    if (!first) return null;

    // Hold the newest integrated pose during a worker stall. Never extrapolate contact.
    const time = clamp(Math.max(this.cursor, now - this.origin - PRESENTATION_DELAY_MS), first.time, last.time);
    this.cursor = time;
    while (this.samples.length > 1 && this.samples[1].time <= time) this.samples.shift();
    const [older, newer] = this.samples;
    if (!newer || time === older.time) return older.pose;

    const ratio = (time - older.time) / (newer.time - older.time);
    const lerp = (a, b) => a + (b - a) * ratio;
    const pose = { park: older.pose.park };
    // Position, yaw and wheel spin in the physics snapshots are continuous and unwrapped.
    for (const key of scalarChannels) pose[key] = lerp(older.pose[key], newer.pose[key]);
    for (const key of arrayChannels) pose[key] = older.pose[key].map((value, i) => lerp(value, newer.pose[key][i]));
    return pose;
  }
}

function samePose(a, b) {
  const near = (x, y) => Math.abs(x - y) < 1e-7;
  return b && a.park === b.park && scalarChannels.every(key => near(a[key], b[key])) &&
    arrayChannels.every(key => a[key].every((value, i) => near(value, b[key][i])));
}

export async function install({ viewer, root, read, base, output, help, mode }) {
  const galleryModule = name => import(new URL('src/' + name, document.baseURI).href);
  const [{ PythonLogic }, { inputs }, rig] = await Promise.all([
    galleryModule('python-logic.js'), galleryModule('vehicle-host.js'), read('rig.json'),
  ]);
  function part(name) {
    const node = root.getObjectByName(name);
    if (!node) throw Error('Missing vehicle part: ' + name);
    return { node, position: node.position.clone(), rotation: node.quaternion.clone() };
  }
  const chassis = part('Car');
  const doors = doorNames.map(name => ({ ...part(name), axis: new Vector3(...rig[name]).normalize() }));
  const runningGear = wheels.map(name => ({
    carrier: part(name + '_Carrier'), steering: part(name + '_Steer'), rolling: part(name + '_Spin'),
  }));
  const controlled = [
    chassis, ...doors, ...runningGear.flatMap(wheel => [wheel.carrier, wheel.steering, wheel.rolling]),
  ];
  const rotation = new Quaternion(), target = new Vector3(), eye = new Vector3();
  const presentation = new DrivePresentation();
  let saved, displayed, simulationTimer, cameraDirty = false, ready = false, epoch = 0;
  let orbit, elevation, distance, fixed;
  const controls = output.closest('section');
  const start = controls.querySelector('button');

  // Use the shared lighting and shadow objects at this resolution in both modes.
  viewer.lighting.setWeather({ shadowMapSize: 2048 });
  help.textContent = 'W/S accelerate, brake and reverse · A/D steer · Space parking brake · Q/E left/right door · Move mouse to look · Scroll to zoom · F lock/unlock follow angle · Esc inspect';
  output.textContent = 'Loading driving physics…';

  const logic = new PythonLogic({
    module: new URL('sim_adapter.py', base).href,
    dependencies: { 'motion.py': new URL('motion.py', base).href },
  }, snapshot => {
    ready = true;
    if (!mode.active) {
      output.textContent = 'Ready to drive';
      return;
    }
    if (snapshot.epoch !== epoch) return;
    presentation.push(snapshot.car, performance.now());
  }, problem => {
    ready = false;
    mode.set(false);
    start.disabled = true;
    output.textContent = 'Driving unavailable: ' + problem;
  });

  function send() {
    logic.event('controls', Array.from(input.keys, code => keymap[code]).filter(Boolean));
  }
  const input = inputs(mode, viewer.canvas, {
    codes: [...Object.keys(keymap), 'KeyQ', 'KeyE', 'KeyF'],
    key(code) {
      if (code === 'KeyF') {
        fixed = !fixed;
        cameraDirty = true;
      }
      if (code === 'KeyQ' || code === 'KeyE') logic.event('door', code === 'KeyQ' ? 0 : 1);
    },
    send,
    mouse(event) {
      if (fixed || (document.pointerLockElement !== viewer.canvas && event.target !== viewer.canvas)) return;
      orbit += event.movementX * .007;
      elevation = clamp(elevation + event.movementY * .005, .08, 1.22);
      cameraDirty = true;
    },
    wheel(event) {
      distance = clamp(distance * Math.exp(Math.sign(event.deltaY) * .1), 3.2, 14);
      cameraDirty = true;
    },
  });

  function rotate(binding, axis, radians) {
    binding.node.quaternion.copy(binding.rotation).multiply(rotation.setFromAxisAngle(axis, radians));
  }
  function render(state) {
    const car = chassis.node;
    car.rotation.set(state.roll, state.yaw, state.pitch, 'YXZ');
    car.position.copy(center).applyQuaternion(car.quaternion).negate();
    car.position.x += wrapCourse(state.x);
    car.position.y += state.height;
    car.position.z += wrapCourse(state.z);
    runningGear.forEach(({ carrier, steering, rolling }, i) => {
      carrier.node.position.copy(carrier.position).addScaledVector(vertical, state.travel[i]);
      rotate(steering, vertical, state.wheel_steer[i]);
      rotate(rolling, axle, state.spin[i]);
    });
    doors.forEach((door, i) => rotate(door, door.axis, state.door[i]));

    car.updateWorldMatrix(true, false);
    target.copy(center).applyMatrix4(car.matrixWorld);
    target.y += .53;
    const axis = car.matrixWorld.elements;
    const angle = Math.atan2(-axis[2], axis[0]) + orbit;
    const radius = distance * Math.cos(elevation);
    eye.set(radius * Math.cos(angle), distance * Math.sin(elevation), -radius * Math.sin(angle)).add(target);
    viewer.setCamera({ position: eye.toArray(), target: target.toArray(), fov: 44, near: .07, far: 1600 });
    const gear = state.park ? 'P' : state.speed < -.015 ? 'R' : 'D';
    const status = gear + ' · ' + Math.round(Math.abs(state.speed) * 3.6) + ' km/h · ' +
      distance.toFixed(1) + ' m · ' + (fixed ? 'Angle locked' : 'Free look');
    if (output.textContent !== status) output.textContent = status;
  }

  function restart(angles) {
    presentation.reset();
    displayed = null;
    cameraDirty = true;
    logic.event('reset', { epoch: ++epoch, doors: angles });
    send();
  }

  return {
    input,
    setMode(enabled) {
      if (enabled) {
        saved = {
          camera: viewer.savePose(),
          parts: controlled.map(({ node }) => [node, node.position.clone(), node.quaternion.clone()]),
        };
        orbit = .35;
        elevation = .27;
        distance = 6.7;
        fixed = false;
        restart(doors.map(({ node, rotation }) => node.quaternion.angleTo(rotation)));
        // Keep worker requests moving when the shared GPU renderer skips a frame.
        simulationTimer = setInterval(() => logic.update(), 16);
        output.textContent = ready ? 'Starting drive…' : 'Loading driving physics…';
      } else {
        clearInterval(simulationTimer);
        logic.event('stop', null);
        if (saved) {
          for (const [node, position, quaternion] of saved.parts) {
            node.position.copy(position);
            node.quaternion.copy(quaternion);
          }
          viewer.setCamera(saved.camera);
          saved = null;
        }
        presentation.reset();
        displayed = null;
        output.textContent = ready ? 'Ready to drive' : 'Loading driving physics…';
      }
    },
    reset() {
      if (mode.active) {
        input.release();
        restart([0, 0]);
      }
    },
    update() {
      if (!mode.active) return;
      const pose = presentation.sample(performance.now());
      if (!pose || (!cameraDirty && samePose(pose, displayed))) return;
      cameraDirty = false;
      displayed = pose;
      render(pose);
    },
    focus() {
      return mode.active && displayed ? target.toArray() : null;
    },
    dispose() {
      clearInterval(simulationTimer);
      input.dispose();
      logic.dispose();
    },
  };
}

