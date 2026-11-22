"""Deterministic four-contact presentation vehicle. SI units, Y-up and forward -X."""
import math
import json
H=1/120
WB=2.70
MASS=1575.
WHEELS=[(-1.35,.86,.33055),(-1.35,-.86,.33055),(1.35,.85,.3545),(1.35,-.85,.3545)]
AXLE_OFFSET=WB/2
# Keep the inner wheel inside the exported steering joint's ±0.65 rad travel.
MAX_WHEEL_TAN=math.tan(.65)
MAX_STEER=math.atan(MAX_WHEEL_TAN/(1+WHEELS[0][1]*MAX_WHEEL_TAN/WB))
# Preview handling limits, not measured factory tyre characteristics.
MAX_LATERAL_ACCEL=9.0
MAX_LATERAL_JERK=30.0
MIN_DISPLAY_CURVATURE = math.tan(math.radians(6.0))/WB

def steering_curvature_limit(speed):
    return min(math.tan(MAX_STEER)/WB, MAX_LATERAL_ACCEL/max(speed*speed,1.))
def clip(v,a,b):return max(a,min(b,v))
def toward(v,target,delta):return v+clip(target-v,-delta,delta)
def wheel_angles(curvature):
    """Ackermann front-wheel headings for a rear-axle turning radius."""
    angles=[clip(math.atan2(WB*curvature,1-az*curvature),-.65,.65)
            for _,az,_ in WHEELS[:2]]
    return angles+[0.,0.]
TRACK_X0,TRACK_X1,TRACK_Z0,TRACK_Z1=-80.,-5.,-6.,6.
TRACK_SPACING=.25
def smooth(value):
    value=clip(value,0.,1.)
    return value*value*(3.-2.*value)
def track_vertex(x,z):
    width=smooth((5.-abs(z))/1.5)
    bank=.48*smooth((-x-16.)/5.)*smooth((x+37.)/6.)
    jump=.49*smooth((-x-49.)/8.)*smooth((x+60.)/1.25)
    bump=.075*smooth((-x-7.)/1.)*smooth((x+10.)/1.)*smooth((z-.05)/.50)*smooth((2.4-z)/.6)
    return width*(bank+jump)+bump
def ground_height(x,z):
    """Interpolate the same triangle mesh shown under the wheels in the viewer."""
    x=(x+216.)%432.-216.
    z=(z+216.)%432.-216.
    if not (TRACK_X0<x<TRACK_X1 and TRACK_Z0<z<TRACK_Z1):return 0.
    h=TRACK_SPACING
    x0=math.floor(x/h)*h;z0=math.floor(z/h)*h
    u=(x-x0)/h;v=(z-z0)/h
    a=track_vertex(x0,z0);c=track_vertex(x0+h,z0+h)
    if u>=v:return (1.-u)*a+(u-v)*track_vertex(x0+h,z0)+v*c
    return (1.-v)*a+(v-u)*track_vertex(x0,z0+h)+u*c
class Vehicle:
    def __init__(self):
        self.x=2.44;self.z=self.speed=self.yaw=self.steer=self.pitch=self.roll=0.
        self.turn_curvature=0.
        self.height=self.vy=self.pitchrate=self.rollrate=self.accum=0.
        self.door=[0.,0.];self.wanted=[0.,0.];self.spin=[0.]*4
        self.travel=[0.]*4;self.contact=[True]*4;self.reverse_delay=0.
        self.park=False;self.frames=0
    def toggle(self,index):self.wanted[index]=0. if self.wanted[index] else 1.19
    def step(self,dt,keys=None,terrain=None):
        keys=keys or {};terrain=terrain or (lambda x,z:0.)
        if not math.isfinite(dt) or dt<0:raise ValueError('dt must be finite and nonnegative')
        self.accum+=min(dt,.25)
        while self.accum>=H-1e-12:
            self.accum=max(0.,self.accum-H); self.integrate(keys,terrain)
        return self.state()
    def integrate(self,k,ground):
        self.frames+=1;v=self.speed;forward=bool(k.get('w'));reverse=bool(k.get('s'))
        if k.get('space'):self.park=True
        elif forward or reverse:self.park=False
        direction=int(forward)-int(reverse)
        grip=sum(self.contact)/4
        if self.park or forward and reverse:
            v=toward(v,0,11.5*grip*H);self.reverse_delay=.24
        elif direction and direction*v<-.001:
            v=toward(v,0,10.8*grip*H);self.reverse_delay=.24
        elif direction and self.reverse_delay>0 and abs(v)<.01:
            v=0.;self.reverse_delay-=H
        elif direction:
            drive=(5.8/(1+abs(v)/30)) if direction>0 else 2.6
            v=clip(v+direction*drive*grip*H,-8.,75.)
        if not self.park and grip>0:
            slope=(ground(self.x-.10*math.cos(self.yaw),self.z+.10*math.sin(self.yaw))-ground(self.x+.10*math.cos(self.yaw),self.z-.10*math.sin(self.yaw)))/.20
            if abs(v)>.015 or direction or abs(slope)>.04:v-=clip(slope,-.55,.55)*9.81*grip*H
        v=toward(v,0,(.19+v*v*.00048)*grip*H)
        accel=(v-self.speed)/H;self.speed=v
        # Limit curvature before filtering so the response stays quick on release
        # and countersteer, even when the lateral acceleration cap dominates.
        command=int(bool(k.get('a')))-int(bool(k.get('d')))
        curvature_limit=steering_curvature_limit(v)
        target=math.atan(WB*curvature_limit)*command
        steer_rate=1.45/(1+abs(v)/45.)
        rack_curvature=math.tan(toward(self.steer,target,steer_rate*H))/WB
        self.turn_curvature=clip(toward(self.turn_curvature,rack_curvature,
            MAX_LATERAL_JERK/max(v*v,1.)*H),-curvature_limit,curvature_limit)
        self.steer=math.atan(WB*self.turn_curvature)
        headings=wheel_angles(self.turn_curvature)
        yawrate=v*self.turn_curvature*grip
        old_yaw=self.yaw
        half_turn=yawrate*H/2
        self.yaw+=2*half_turn
        c=math.cos(self.yaw);s=math.sin(self.yaw)
        # Follow the arc at the rear axle, then recover the body midpoint.
        rear_travel=v*H*(math.sin(half_turn)/half_turn if half_turn else 1.)
        self.x-=math.cos(old_yaw+half_turn)*rear_travel+AXLE_OFFSET*(c-math.cos(old_yaw))
        self.z+=math.sin(old_yaw+half_turn)*rear_travel+AXLE_OFFSET*(s-math.sin(old_yaw))
        total=0.;pt=0.;rt=0.;spring=65000.;preload=MASS*9.81/(4*spring)
        for i,(ax,az,r) in enumerate(WHEELS):
            wx=self.x+c*ax+s*az;wz=self.z-s*ax+c*az
            surface=ground(wx,wz);zbody=self.height+ax*math.sin(self.pitch)-az*math.sin(self.roll)
            delta=surface-zbody
            self.travel[i]=clip(delta,-.095,.088);self.contact[i]=delta>=-preload
            rate=self.vy+ax*self.pitchrate-az*self.rollrate
            # Contact-relative damping includes the terrain velocity beneath each wheel.
            wheel_forward=v-az*yawrate
            wheel_left=(AXLE_OFFSET-ax)*yawrate
            dx=-c*wheel_forward+s*wheel_left
            dz=s*wheel_forward+c*wheel_left
            slope_rate=(ground(wx+dx*H,wz+dz*H)-surface)/H
            force=clip(spring*(preload+delta)-3900*(rate-slope_rate),0,36000) if self.contact[i] else 0.
            total+=force;pt+=ax*force;rt-=az*force
            rolling_speed=wheel_forward*math.cos(headings[i])+wheel_left*math.sin(headings[i])
            self.spin[i]+=rolling_speed/r*H
        self.vy+=(total/MASS-9.81)*H;self.height+=self.vy*H
        self.pitchrate+=(pt-MASS*.36*accel-340*self.pitchrate)/2650*H
        self.rollrate+=(rt-MASS*.36*v*yawrate-310*self.rollrate)/860*H
        self.pitch=clip(self.pitch+self.pitchrate*H,-.65,.65);self.roll=clip(self.roll+self.rollrate*H,-.65,.65)
        for i in range(2):self.door[i]=toward(self.door[i],self.wanted[i],1.45*H)
    def state(self):
        state={k:getattr(self,k) for k in ('x','z','speed','yaw','steer','turn_curvature','height','pitch','roll','door','spin','travel','contact','park','frames')}
        # Preserve the filtered rack's progression and Ackermann split, while
        # giving full input a readable display angle when the path cap is small.
        # The tyre-roll and chassis calculations use the original curvature.
        path_limit=steering_curvature_limit(self.speed)
        display_scale=max(1.,MIN_DISPLAY_CURVATURE/path_limit)
        state['wheel_steer']=wheel_angles(self.turn_curvature*display_scale)
        return state
    def checkpoint(self):
        """Full integrator state, including pending input delays and damping rates."""
        return json.dumps(vars(self),allow_nan=False)
    @classmethod
    def resume(cls,checkpoint):
        obj=cls();values=json.loads(checkpoint)
        legacy='turn_curvature' not in values
        expected=set(vars(obj))-({'turn_curvature'} if legacy else set())
        if set(values)!=expected:raise ValueError('Incomplete vehicle checkpoint')
        for name,default in vars(obj).items():
            if name=='turn_curvature' and legacy:continue
            value=values[name]
            if isinstance(default,list):
                if not isinstance(value,list) or len(value)!=len(default):raise ValueError(name)
                data=value
            else:data=[value]
            if not all(isinstance(n,(int,float)) and math.isfinite(n) for n in data):raise ValueError(name)
        if legacy:
            limit=MAX_LATERAL_ACCEL/max(values['speed']**2,1.)
            values['turn_curvature']=clip(math.tan(values['steer'])/WB,-limit,limit)
        # Earlier saves could have an independently wound-up visual rack.
        values['steer']=math.atan(WB*values['turn_curvature'])
        vars(obj).update(values)
        return obj
vehicle=Vehicle()
def advance(dt,keys):return json.dumps(vehicle.step(dt,json.loads(keys),ground_height),allow_nan=False)

