import numpy as np


class World(object):
    # nasa convention is to use Z axis as spin axis
    def __init__(self, is3D=False):
        self.is3D = is3D
        self.M = None
        self.R = None
        self.c = np.inf
        self.scale_height = 0
        self.colour = '#575757'
    
    @staticmethod
    def rho(alt):
        return 0

    @property
    def mu(self):
        return 6.674e-11 * self.M

    def g(self, alt):
        return self.mu / (self.R + alt)**2


class Earth(World):
    def __init__(self, is3D=False):
        super().__init__(is3D)
        self.M = 5.9724e24
        self.R = 6371.0e3
        self.scale_height = 8.5e3
        self.c = 330
        self.colour = '#00d681'

    @staticmethod
    def rho(alt):        
        if alt > 25000:
            T = -131.21 + 0.00299*alt
            p = 2.488 * ((T+273.1)/216.6)**-11.388
            if alt > 40000:     # not strictly in NASA earth model
                                # included due to scaling for orbital speeds 
                                # causes massive aerodynamic forces
                p /= 1 + (alt-40000)/8000
        elif alt > 11000:
            T = -56.46
            p = 22.65 * np.e**(1.73-0.000157*alt)
        else:
            T = 15.04 - 0.00649*alt
            p = 101.29 * ((T+273.1)/288.08)**5.256
        return p / (0.2869 * (T+273.1))


class Mars(World):
    def __init__(self, is3D=False):
        super().__init__(is3D)
        self.M = 0.64171e24
        self.R = 3389.5e3
        self.scale_height = 11.1e3
        self.c = 244
        self.colour = '#bf8d04'
    
    @staticmethod
    def rho(alt):
        if alt > 20000:
            return 0
        elif alt > 7000:
            T = -23.4 - 0.00222*alt
            p = 0.699 * np.e**(-0.00009*alt)
        else:
            T = -31 - 0.000998*alt
            p = 0.699 * np.e**(-0.00009*alt)
        return p / (0.1921 * (T+273.1))


# import matplotlib.pyplot as plt 

# p = Earth()
# alts = np.linspace(0, 100e3, 100)
# vfunc = np.vectorize(p.rho)
# plt.plot(alts, vfunc(alts))
# plt.ylim(0,0.1)
# plt.show()