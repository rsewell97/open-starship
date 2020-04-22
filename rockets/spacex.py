import numpy as np

from rockets.rocket import Rocket


class Starship(Rocket):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.radius = 4.5
        self.height = 50
        self.propellant_mass = 3.3e6
        self.dry_mass = 120e3
        self.max_thrust = 2e5 * 3
        self.Isp = 386
        self.max_v_gimbal = 30
        self.max_gimbal = 20
        self.min_throttle = 0#0.1 / 3
        self.fuel_level = 0.2
        self.reenters_engine_first = False
        self.rated_for_human_reentry = True

        self.front_wing_area = 19.4
        self.rear_wing_area = 38.6
        self.wing_thickness = 0.5

        if self.planet.is3D:
            self.control_vec = np.zeros(7)
        else:
            self.control_vec = np.zeros(4)

    def update(self, dt):
        wing_angs = np.zeros((2,2)) #* np.pi/2  # ang=0 is perpendicular to hull
        # [[front_left, front_right],
        # [rear_left, rear_right]]
        gimbal_cntrl = np.ones(2) * np.pi/2
        throttle = 0
        wing_angs = np.clip(wing_angs, 0, np.pi/2)

        x, y, z = np.eye(3)

        common_factor =  0.5* self.planet.rho(self.altitude) * self.speed**2
        
        # Starship drags can be decomposed to those perpendicular to the hull
        # and those parallel to the hull.
        front_drag = 0.1 * self.wing_thickness*2 * np.cos(self.attackAngle) * common_factor
        rear_drag =  0.1 * self.wing_thickness*3 * np.cos(self.attackAngle) * common_factor
        # assume front and rear drags behave as a linear system
        folded_wing_torque_effect = (2*front_drag*np.cos(np.mean(wing_angs[0])) 
                                    + 2.5*rear_drag*np.cos(np.mean(wing_angs[1])))*y
        total_drag = (front_drag + rear_drag) * -x

        # Clearly perpendicular lift forces dominate
        front_lift = 0.6 * self.front_wing_area* np.cos(wing_angs[0]) * np.sin(self.attackAngle) * common_factor
        rear_lift =  0.6 * self.rear_wing_area * np.cos(wing_angs[1]) * np.sin(self.attackAngle) * common_factor

        # model pitch torque as difference between lateral drags with some restoring torque     
        pitch_torque = (22*np.mean(front_lift) - 18*np.mean(rear_lift))*y

        # assumed to only depend on forces perpendicular to hull
        roll_torque = (front_lift[0]+rear_lift[0] -front_lift[1]-rear_lift[1])*6*x
        yaw_torque = (front_lift[0]+rear_lift[1] -front_lift[1]-rear_lift[0])*z

        # TODO: compute lift forces

        force_sum = total_drag
        torque_sum = folded_wing_torque_effect

        self.acc = self.acc + force_sum / self.mass
        self.ddrot = self.ddrot + torque_sum / self.inertia


class F9(Rocket):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.radius = 3.7/2
        self.height = 40
        self.propellant_mass = 420e3
        self.dry_mass = 30e3
        self.max_thrust = 854e3 * 3
        self.Isp = 282
        self.max_v_gimbal = 30
        self.max_gimbal = 20
        self.min_throttle = 0#0.1 / 3
        self.fuel_level = 0.15
        self.gf_eff_area = 1.8

        if self.planet.is3D:
            self.control_vec = np.zeros(7)
        else:
            self.control_vec = np.zeros(4)

    def update(self, dt):
        gf_angs = np.ones((2,2)) 
        grid_fin_angs = np.clip(-20*np.pi/180, 0, 20*np.pi/180)
        # [[y, -y, z, -z]]

        gimbal_cntrl = np.ones(2)
        throttle = 0
        
        x, y, z = np.eye(3)
        gf_axes = np.array([y, -y, z, -z])
        force_dir = np.array([z, -z, -y, y])

        common_factor =  0.5* self.planet.rho(self.altitude) * self.gf_eff_area * self.speed**2
        gf_forces = 0.4 * common_factor * np.sin(gf_angs.ravel())
        f = np.asarray([gf_forces[i] * force_dir[i] for i in range(4)])
        gf_torques = np.cross(x*self.height/2 + gf_axes*(self.radius+1), f)

        gf_drag = 4 *0.5 * common_factor * x

        force_sum = gf_drag
        torque_sum = np.sum(gf_torques, axis=0)

        self.acc = self.acc + force_sum / self.mass
        self.ddrot = self.ddrot + torque_sum / self.inertia
    
    def typicalEntry(self):
        num = int(input('Enter 1 for RTLS, 2 for ADSL: '))
        if num == 1:
            self.startAt([0, 120e3+self.planet.R, 0], [200, 0, 0])
        elif num == 2:          #TODO: these are guesstimates based on online graphs
            self.startAt([0, 120e3+self.planet.R, 0], [200, 0, 0])