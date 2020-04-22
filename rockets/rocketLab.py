import numpy as np

from rockets.rocket import Rocket


class Electron(Rocket):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.radius = 0.6
        self.height = 17
        self.propellant_mass = 9.25e3
        self.dry_mass = 0.95e3
        self.max_thrust = 162e3 * 9
        self.Isp = 303
        self.max_v_gimbal = 30
        self.max_gimbal = 20
        self.min_throttle = 0.1 / 9
        self.fuel_level = 0.2

    def update(self, dt):
        # TODO
        return

    def typicalEntry(self):
        self.startAt([0, 120e3+self.planet.R, 0], [200, 0, 0])
