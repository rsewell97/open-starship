import time
import multiprocessing as mp

import numpy as np
from scipy.spatial.transform import Rotation

from world import Earth


class Rocket(object):
    def __init__(self, planet=Earth()):
        self.planet = planet

        self.propellant_mass = 1e6
        self.dry_mass = 40e3
        self.max_thrust = 2e5 * 2
        self.Isp = 350
        self.fuel_level = 0.08
        self.radius = 3.5
        self.height = 40
        self.max_v_gimbal = 30
        self.max_gimbal = 20
        self.min_throttle = 0
        self.reenters_engine_first = True
        self.rated_for_human_reentry = False

        # IN WORLD COORDINATES
        self.pos = np.array([0, self.planet.R, 0])
        self.vel = np.zeros(3)
        self.rot = Rotation.identity()
        self.drot = np.zeros(3)

        # IN LOCAL COORDINATES
        self.acc = np.zeros(3)
        self.ddrot = np.zeros(3)
        self.gimbal = np.zeros(3)       # y, z, x
        self.v_gimbal = np.zeros(3)     # y, z, x
        
        self.clock = 0
        self.plotter = None

        if self.planet.is3D:
            self.control_vec = np.zeros(3)
        else:
            self.control_vec = np.zeros(2)

    @property
    def mass(self):
        return self.dry_mass + self.fuel_level*self.propellant_mass

    @property
    def inertia(self):
        # assume Ixx is uniform disc lamina
        Ixx = 0.5*self.mass*self.radius**2
        # assume Iyy is uniform rod
        Iyy = self.mass*(self.height**2) / 12
        # likewise for Izz
        Izz = Iyy
        # assume no gyration effects
        return np.array([Ixx, Iyy, Izz])

    @property
    def finenessRatio(self):
        return self.height/self.radius/2

    @property
    def deltaV(self):
        return self.Isp * np.log(self.mass/self.dry_mass)

    @property
    def altitude(self):
        return self.orbitalDist - self.planet.R

    @property
    def latitude(self):
        return np.arcsin(self.pos[2] / self.orbitalDist) * 180/np.pi

    @property
    def longitude(self):
        lon = np.arcsin(self.pos[1] / np.linalg.norm(self.pos[:2])) * 180/np.pi
        if self.pos[0] >= 0:
            return lon
        else:
            if lon > 0:
                return 180-lon
            else:
                return -180+lon

    @property
    def horizonAxes(self):
        y = self.pos/self.orbitalDist       # radial axis
        z = -self.orbitalAngMomentum/np.linalg.norm(self.orbitalAngMomentum)   #norm to orbital plane
        x = np.cross(y, z)                  # forward horizon dir
        return np.array([x, y, z])

    @property
    def localAxes(self):
        return self.toWorld(np.eye(3))

    @property
    def eulerAngles(self):
        l = self.toLocal(self.horizonAxes)
        return Rotation.from_matrix(l).as_euler('yzx', degrees=True)

    @property
    def heading(self):
        return self.localAxes[0]

    @property
    def attackAngle(self):
        # in range 0 -> pi
        return np.arccos(np.clip(np.dot(self.vel/self.speed, self.heading), -1, 1))

    @property
    def speed(self):
        return np.linalg.norm(self.vel)

    @property
    def machNum(self):
        return self.speed / self.planet.c

    @property
    def Gs(self):
        return np.linalg.norm(self.acc) / self.planet.g(self.altitude)

    @property
    def radialSpeed(self):
        return np.dot(self.vel, self.horizonAxes[1])

    @property
    def tangentialSpeed(self):
        return np.linalg.norm(self.tangentialVel)

    @property
    def tangentialVel(self):
        return np.cross(self.horizonAxes[1], self.orbitalAngMomentum/(self.mass*self.orbitalDist))

    @property
    def orbitalAngMomentum(self):
        return np.cross(self.pos, self.vel*self.mass)

    @property
    def orbitalDist(self):
        return np.linalg.norm(self.pos)

    @property
    def currentTraj(self):
        b = 2*self.radialSpeed / self.planet.g(self.altitude)
        c = -2*self.altitude / self.planet.g(self.altitude)
        pos_time_to_hit_ground = (-b + (b**2 - 4*c) ** 0.5) / 2
        neg_time_to_hit_ground = (-b - (b**2 - 4*c) ** 0.5) / 2

        ts = np.linspace(2*neg_time_to_hit_ground, 2 *
                         pos_time_to_hit_ground, 100)
        ts = np.expand_dims(ts, 1)

        w = self.radialSpeed * self.horizonAxes[1]
        x = 0.5*self.planet.g(self.altitude) * self.horizonAxes[1] * 1.4

        traj = np.apply_along_axis(lambda t: self.pos + (w+self.tangentialVel)*t - x*t**2, 1, ts)
        return traj

    @property
    def hullTemperature(self):
        # TODO
        return None

    @property
    def states(self):
        horizon_ang_vel = self.drot + self.toLocal(self.orbitalAngMomentum/(self.mass*self.orbitalDist**2))
        return [
            self.altitude, self.radialSpeed, self.tangentialSpeed,
            self.eulerAngles.ravel(), horizon_ang_vel.ravel(), self.gimbal.ravel()
        ]

    def calcUpdate(self, dt, gimbal_cntrl=[5, 0], throttle=0.):
        # perform control actions and calculate rocket mechanics
        # assume rocket is modelled as a featureless cylinder

        # TRANSFORM INTO LOCAL COORDINATES
        x,y,z = np.eye(3)
        v = self.toLocal(self.vel)
        
        # FORCES
        # thrust
        gimbal_cntrl = np.deg2rad(gimbal_cntrl)
        gimbal = Rotation.from_rotvec([0, gimbal_cntrl[0], gimbal_cntrl[1]])
        thrust = gimbal.apply(throttle * self.max_thrust * x)
        self.fuel_level -= np.linalg.norm(thrust) / (self.Isp * self.planet.g(self.altitude) * self.propellant_mass)

        # aero (cylinder)
        common_factor = 0.5 * self.planet.rho(self.altitude) * self.speed**2
        drag = -x*0.62*common_factor* np.pi*self.radius**2 * np.cos(self.attackAngle)

        # lift is perpendicular to ship, calculated using eqn (32): https://apps.dtic.mil/dtic/tr/fulltext/u2/603829.pdf
        lift_normal = self.normalised(np.cross(x,  np.cross(x, v)))
        lift = lift_normal*0.1*common_factor* 2*self.radius*self.height * np.sin(self.attackAngle)**3

        # TORQUES
        # aero restoring torque
        restoring_axis = self.normalised(np.cross(x, v))
        aero_torque = 0.05 * self.height/8 * common_factor * restoring_axis * self.finenessRatio

        # gimballing torque
        thrust_torque = np.cross(-self.height*x/2, thrust)
        
        force_sum = drag + lift + np.dot(thrust, x)
        torque_sum = thrust_torque + aero_torque               # vec

        self.acc = force_sum / self.mass       # local coord
        self.ddrot = torque_sum / self.inertia    # local coord 

    def _update(self, dt):
        self.calcUpdate(dt)
        self.update(dt)

        grav = self.planet.g(self.altitude) * self.pos/self.orbitalDist
        self.vel = self.vel + (self.toWorld(self.acc)-grav)*dt
        self.pos = self.pos + self.vel*dt

        self.drot = self.drot + self.ddrot*dt 
        upd = Rotation.from_rotvec(self.drot*dt)
        self.rot = Rotation.from_matrix(upd.apply(self.rot.as_matrix()))

        self.clock += dt
        if self.plotter is not None:
            to_send = {}
            for k, v in self.plotting_info.items():
                if type(v) == list:
                    to_send[k] = [getattr(self, i) for i in v]
                else:
                    to_send[k] = getattr(self, v)
            self.plotter.update(to_send)

    def update(self, dt):
        # base function to add specific rocket dynamics: modiry self.acc and self.ddrot here
        pass

    def runSim(self, dt=1, T=None):
        if T is None:
            T = np.inf

        prev_time = time.time()
        while self.speed != 0:
            self._update(dt)
            if not self.failConditions():
                break
            elif self.clock > T:
                break
            print('\r', end='')
            print(f'Simulation speed: {dt/(time.time()-prev_time):.1f}x, update freq: {1/(time.time()-prev_time):.1f}Hz', end='', flush=True)
            prev_time = time.time()

    def startAtApoapsis(self, apoapsis=100e3, periapsis=-20e5, inclination=0):
        apoapsis += self.planet.R
        periapsis += self.planet.R
        a = (apoapsis + periapsis) / 2
        # e = ((apoapsis - a) + (periapsis - a)) / (2*a)
        v = (self.planet.mu*(2/apoapsis - 1/a)) ** 0.5

        inclination = np.deg2rad(inclination)
        self.pos = np.array([0, apoapsis*np.cos(inclination), apoapsis*np.sin(inclination)])
        self.vel = np.cross(self.pos, np.array([0, -np.sin(inclination), np.cos(inclination)]))
        self.vel = self.vel/self.speed * v
        self.rot = Rotation.from_euler('yzx', [self.reenters_engine_first*np.pi, 
                                                0, -np.pi/2+inclination])
        world_ang_vel = self.orbitalAngMomentum/(self.mass*self.orbitalDist**2)
        self.drot = -self.toLocal(world_ang_vel)

    def startFreefall(self, altitude):
        self.pos = np.array([0, altitude+self.planet.R, 0])
        self.vel = np.array([0, -0.1, 0.1])
        self.rot = Rotation.from_euler('yzx', [self.reenters_engine_first*np.pi/2 + 0.01, 
                                                -np.pi/2, 0]).as_quat()
        self.drot = np.zeros(3)

    def startAt(self, pos, vel):
        self.pos = np.asarray(pos)
        self.vel = np.asarray(vel)

    def typicalEntry(self):
        raise NotImplementedError("This will be added in a future update")

    def failConditions(self):
        # _tmp = abs(self.reenters_engine_first*np.pi - self.attackAngle)
        # if _tmp > 2*np.pi/3 and self.speed > 3000:
        #     print('\nFlamey end is not pointing down')
        #     return False    # pointing wrong-end-first
        # if self.speed > 7900:
        #     print('\nYou made a meteor')
        #     return False    # nothing has survived reentry at this speed
        # if self.Gs > 8 and self.rated_for_human_reentry:
        #     print(f'\n{self.Gs:.1f}G - congrats everyone is unconscious')
        #     return False    # too much G force for occupants to handle
        if self.altitude <= 0 and self.speed > 2:
            print(f'\nRUD on landing pad {self.speed:.1f}m/s')
            return False    # crash land       
        return True

    def attachPlotter(self, obj):
        self.plotter = obj
        self.plotting_info = self.plotter.reqDataFormat()

    def toLocal(self, vec):
        return self.rot.inv().apply(vec)

    def toWorld(self, vec):
        return self.rot.apply(vec)

    @staticmethod
    def normalised(v):
        n = np.sqrt(np.dot(v,v))
        if n == 0:
            return np.zeros_like(v)
        else:
            return v / np.sqrt(np.dot(v,v))