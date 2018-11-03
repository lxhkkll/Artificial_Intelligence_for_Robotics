# coding=utf-8
"""
https://www.itcodemonkey.com/article/5722.html
"""
import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import cubic_spline
import seaborn
from calc_polynomial import *

# Parameter
MAX_SPEED = 50.0 / 3.6  # 最大速度 [m/s]
MAX_ACCEL = 2.0  # 最大加速度[m/ss]
MAX_CURVATURE = 1.0  # 最大曲率 [1/m]
MAX_ROAD_WIDTH = 7.0  # 最大道路宽度 [m]
D_ROAD_W = 1.0  # 道路宽度采样间隔 [m]
DT = 0.2  # Delta T [s]
MAXT = 5.0  # 最大预测时间 [s]
MINT = 4.0  # 最小预测时间 [s]
TARGET_SPEED = 30.0 / 3.6  # 目标速度（即纵向的速度保持） [m/s]
D_T_S = 5.0 / 3.6  # 目标速度采样间隔 [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 2.0  # robot radius [m]

# 损失函数权重
KJ = 0.1
KT = 0.1
KD = 1.0
KLAT = 1.0
KLON = 1.0


class Frenet_path:
    def __init__(self):
        self.t = []
        # 横侧向 calc_frenet_paths
        self.d = []     #d
        self.d_d = []   #d'
        self.d_dd = []  #d''
        self.d_ddd = [] #d'''
        # 纵向 calc_frenet_paths
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        # 损失 calc_frenet_paths
        self.cd = 0.0   # 横向损失
        self.cv = 0.0   # 纵向损失
        self.cf = 0.0   # 总损失
        # 全局 calc_global_paths
        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    """

    :param c_speed: 当前车速度
    :param c_d:     当前横向位移
    :param c_d_d:   当前横向速度
    :param c_d_dd:  当前横向加速度
    :param s0:      当前所在位置
    :return:
    """
    frenet_paths = []

    # 采样，并对每一个目标配置生成轨迹
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):  # 路宽(di 为目标的位置)

        # 横向动作规划
        for Ti in np.arange(MINT, MAXT, DT):                         # 时间
            fp = Frenet_path()
            # 计算出关于目标配置di，Ti的横向多项式
            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # 纵向速度规划 (速度保持)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2
                # 横向的损失函数
                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1] ** 2
                # 纵向的损失函数
                tfp.cv = KJ * Js + KT * Ti + KD * ds
                # 总的损失函数为d 和 s方向的损失函数乘对应的系数相加
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    """
    计算全局位置
    :param fplist:
    :param csp:
    :return:
    """
    for fp in fplist:

        # 计算全局位置
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx ** 2 + dy ** 2))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    """
    静态障碍，检查碰撞
    :param fp: 采样路径
    :param ob: 障碍
    :return:
    """
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    """
    检查约束条件
    :param fplist: 采样路径
    :param ob: 障碍物
    :return:
    """
    okind = []
    for i in range(len(fplist)):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):             # 最大速度检查
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):     # 最大加速度检查
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):    # 最大曲率检查
            continue
        elif not check_collision(fplist[i], ob):
            continue

        okind.append(i)

    return [fplist[i] for i in okind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)

    # 找到损失最小的轨迹
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp

    return bestpath


def generate_target_course(x, y):
    """
    生成全局路径信息
    :param x:
    :param y:
    :return:
    """
    csp = cubic_spline.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)                       # x
        ry.append(iy)                       # y
        ryaw.append(csp.calc_yaw(i_s))      # 斜率
        rk.append(csp.calc_curvature(i_s))  # 曲率

    return rx, ry, ryaw, rk, csp


def main():
    # 路线
    wx = [0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0]
    wy = [0.0, -4.0, 1.0, 6.5, 8.0, 10.0, 6.0]
    # 障碍物列表
    ob = np.array([[20.0, 10.0],
                   [30.0, 6.0],
                   [30.0, 5.0],
                   [35.0, 7.0],
                   [50.0, 12.0]
                   ])

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy) # 曲线各点x,y,斜率，曲率以及离散曲线

    # 初始状态
    c_speed = 10.0 / 3.6    # 当前车速 [m/s]
    c_d = 2.0               # 当前的d方向位置 [m] d为法向Lateral
    c_d_d = 0.0             # 当前横向速度 [m/s]
    c_d_dd = 0.0            # 当前横向加速度 [m/s2]
    s0 = 0.0                # 当前所在的位置

    area = 20.0             # 动画显示区间 [m]

    for i in range(500):
        path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)

        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]

        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("到达目标")
            break

        plt.cla()
        plt.plot(tx, ty, "r")
        plt.plot(tx, [i+15 for i in ty], "black", markersize=4)
        plt.plot(tx, [i-15 for i in ty], "black", markersize=4)
        plt.plot(ob[:, 0], ob[:, 1], "ob")
        plt.plot(path.x[1:], path.y[1:], "-og")
        plt.plot(path.x[1], path.y[1], "vc")
        plt.xlim(path.x[1] - area, path.x[1] + area)
        plt.ylim(path.y[1] - area, path.y[1] + area)
        plt.title("speed[km/h]:" + str(c_speed * 3.6)[0:4])
        plt.grid(True)
        plt.pause(0.0001)

    plt.grid(True)
    plt.pause(0.0001)
    plt.show()


if __name__ == '__main__':
    main()