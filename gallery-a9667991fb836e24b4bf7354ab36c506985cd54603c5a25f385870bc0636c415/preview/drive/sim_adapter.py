"""Connect Gallery input events to the original fixed-step driving core."""
from motion import Vehicle, ground_height


class Runtime:
    def __init__(self):
        self.car = Vehicle()
        self.epoch = 0
        self.running = False
        self.held = set()
        self.taps = set()

    def step(self, events, elapsed):
        for event, value in events:
            if event == 'reset':
                self.car = Vehicle()
                self.car.door[:] = self.car.wanted[:] = value['doors']
                self.epoch = value['epoch']
                self.running = True
                self.held.clear()
                self.taps.clear()
            elif event == 'stop':
                self.running = False
                self.held.clear()
                self.taps.clear()
            elif self.running and event == 'controls':
                keys = set(value)
                self.taps.update(keys - self.held)
                self.held = keys
            elif self.running and event == 'door':
                self.car.toggle(value)
        previous_frame = self.car.frames
        state = self.car.step(elapsed, dict.fromkeys(self.held | self.taps, True), ground_height) \
            if self.running else self.car.state()
        if state['frames'] != previous_frame:
            self.taps.clear()
        return {'epoch': self.epoch, 'car': state}


def create_runtime(config):
    return Runtime()

