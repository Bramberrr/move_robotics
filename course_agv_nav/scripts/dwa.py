#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = False


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    函数dwa_control定义了一个方法，给定状态，障碍物，目标，调用calc_dynamic_window，计算出一个动态窗口；
    调用calc_final_input，计算出一个该动态窗口所限制的动作空间里一个最佳的动作(v,w)，以及其对应的最佳轨迹。
    """

    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.8  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yawrate = 100.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 1  # [m/ss]
        self.max_dyawrate = 100.0 * math.pi / 180.0  # [rad/ss]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s]
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s]
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 0.5
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_type = RobotType.rectangle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.4  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.3  # [m] for collision check
        self.robot_length = 0.6  # [m] for collision check

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


def motion(x, u, dt):
    """
    motion model
    直线轨迹模型,即不能纵向移动，只能前进和旋转,返回的x代表的机器人的状态，x=[x, y, orientation, v, w]
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    依据机器人的加减速性能限定速度采样空间在一个可行的动态范围内。
    考虑到电机可发挥的有限的加速度，整个搜索空间减少到动态窗口，该窗口仅包含下一个时间间隔内可以达到的速度。
    动态窗口是以实际速度为中心的，它的扩展取决于可以施加的加速度。
    动态窗口外的所有轨迹都不能在下一个时间间隔内达到，因此可以不考虑避障。
    """

    # Dynamic window from robot specification  所有动作(v,w)集合Vs
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    # 原始状态[~, ~, oritentation, v, w]
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yaw_rate min, yaw_rate max]
    # dw = Vs与Vd的交集
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    轨迹产生的原理是，利用motion函数，基于当前机器人的状态x，在未来一段时间内(predict_time)产生一串状态x的序列
    """

    x = np.array(x_init)
    traj = np.array(x)
    time = 0
    # 该循环实现在当前给定的动作下(v,w)， 未来一段时间内生成的轨迹
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)     # 每一个dt内速度不变
        traj = np.vstack((traj, x))     #按照行方向堆叠历经的状态x
        time += config.dt

    return traj


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    输入状态、动态窗口、目标位置、障碍物位置
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):#对搜索空间dw中的v遍历
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):#对w遍历

            trajectory = predict_trajectory(x_init, v, y, config)

            # calc cost

            # TODO here
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            # orientation与目标夹角越小越好
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            # v越大，该项越小，则效果越好；
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
            # 返回的是距离的倒数，所以该值约小，距离越大，越好
            # TODO end
            final_cost = to_goal_cost + speed_cost + ob_cost # 越小轨迹约好

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
    # 在所有动态窗口划出的动作空间(v,w)里，找到一个最好的动作，在这个动作下，未来预测的轨迹评价最好
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
        calc obstacle cost inf: collision
        距离障碍物的距离越大越好
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]     #对历史轨迹中所有的点进行检测
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)
    cost = 0
    # TODO here
    yaw = trajectory[:, 2]
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rot = np.transpose(rot, [2, 0, 1])
    local_ob = ob[:, None] - trajectory[:, 0:2]
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    local_ob = np.array([np.dot(local_ob,x) for x in rot])
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    upper_check = local_ob[:, 0] <= config.robot_length / 2
    right_check = local_ob[:, 1] <= config.robot_width / 2
    bottom_check = local_ob[:, 0] >= -config.robot_length / 2
    left_check = local_ob[:, 1] >= -config.robot_width / 2
    if (np.logical_and(np.logical_and(upper_check, right_check),np.logical_and(bottom_check, left_check))).any():# 如果历史轨迹中任何一点发生了碰撞
        return float("Inf")     # 该cost记为无穷

    cost = np.min(r)
    cost = 1.0 / cost     # 与障碍物距离越近，该值会越大；
    # TODO end
    return cost  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
         行进方向与目标的夹角，夹角越小越好
    """
    cost = 0
    # TODO here
    dx = goal[0] - trajectory[-1, 0]     #堆叠矩阵的最后一行是轨迹末尾点 x
    dy = goal[1] - trajectory[-1, 1]     # y
    error_angle = math.atan2(dy,dx)
    cost_angle = error_angle - trajectory[-1, 2]     # 取出 orientation
    cost = abs(cost_angle)
    # TODO end
    return cost
