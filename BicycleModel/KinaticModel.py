from __future__ import print_function

import math
import matplotlib.pyplot as plt


class KinematicModel(object):
    def __init__(self, x, y, psi, v, f_len, r_len):
        self.x = x
        self.y = y
        self.psi = psi
        self.v = v

        self.f_len = f_len
        self.r_len = r_len

    def get_state(self):
        return self.x, self.y, self.psi, self.v

    def update_state(self, a, delta, dt):
        beta = math.atan((self.r_len / (self.r_len + self.f_len)) * math.tan(delta))

        self.x = self.x + self.v * math.cos(self.psi * beta) * dt
        self.y = self.y + self.v * math.sin(self.psi * beta) * dt
        self.psi = self.psi + (self.v / self.f_len) * math.sin(beta) * dt
        self.v = self.v + a * dt
        return self.x, self.y, self.psi, self.v


if __name__ == '__main__':
    model = KinematicModel(0.0, 0.0, 0.1, 0.0, 10.0, 10.0)
    traject_x = []
    traject_y = []
    psi_l = []
    v_l = []

    a = [0.1, 0.2, 0.3, 0.2, 0.1, 0, 0, 0,0,-0.1, -0.2, 0.1, 0.5, 0.3, 0.4, 0.1, -0.2, -0.3, -0.4, -0.5, -0.5, -0.5, -0.5,-0.5]
    delta = [0,0.1 * math.pi, 0.1 * math.pi, 0.2 * math.pi, 0.1* math.pi, 0.1* math.pi, 0,0,0,0,0,0,0,-0.2*math.pi, -0.3*math.pi, -0.2*math.pi, 0,0,0,0]

    M = 100
    for i in range(M):
        model.update_state(a[i % len(a)], delta[i % len(delta)], 1)
        x, y, psi, v = model.get_state()
        traject_x.append(x)
        traject_y.append(y)
        psi_l.append(psi)
        v_l.append(v)

    figure, (ax1,ax2,ax3) = plt.subplots(3, 1, figsize=(8, 8))
    ax1.plot(traject_x, traject_y)
    ax2.plot(range(M), psi_l)
    ax3.plot(range(M), v_l)
    plt.show()
