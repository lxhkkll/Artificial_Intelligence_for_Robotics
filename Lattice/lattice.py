import numpy as np
import math
import matplotlib.pyplot as plt
from KinaticModel import KinematicModel
from sympy import *

class Lattice:
    def __init__(self, model):
        self.model = model
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 10.0
        self.a = 0.0

        self.figure = plt.figure()
        self.generate_reference_line()
        # plt.xlim(0, 10)
        # plt.ylim(0,10)
        pass

    def coordinate_transfer(self, x0=0.0, y0=0.0):
        x = np.linspace(0.0, 20.0, 10000)
        y = np.log(x + 2) + 6

        d = abs(x - x0 - (y0 - np.log(x + 2) + 6)/(x + 2))/np.sqrt((-x + x0)**2 + (y0 - np.log(x + 2) + 6)**2)
        print(d.min())
        index = int(np.argwhere(d==d.min()))
        res_x, res_y = x[index], y[index]

        plt.plot([x0,res_x], [y0,res_y], 'g', markersize=3)
        print(res_x, res_y)

    def sample_s(self):
        pass

    def sample_l(self):
        pass

    def con_sl(self):
        pass

    def generate_reference_line(self):
        x = np.linspace(0, 20, 10000)
        y = np.log(x + 2) + 6#-0.001 * x**5 - 0.012 * x**4 + 0.03 * x**3 + 0.04* x**2 + x + 6

        plt.plot(x, y, 'r')
        return x, y

if __name__ == '__main__':
    model = KinematicModel(0.0, 0.0, 0.1, 0.0, 10.0, 10.0)
    planner = Lattice(model)
    planner.coordinate_transfer(10.5, 9.5)
    plt.show()