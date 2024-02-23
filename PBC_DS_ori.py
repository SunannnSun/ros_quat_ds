import sys

sys.path.append('./src')

import time
import numpy as np
# from utils import *
from math import sin
import random

from panda_noball import Panda

duration = 50
stepsize = 1e-3

robot = Panda(stepsize)
robot.setControlMode("torque")

lambda1 = 20
lambda2 = 100
lambda3 = 100

for i in range(int(duration / stepsize)):
    if i % 1000 == 0:
        print("Simulation time: {:.3f}".format(robot.t))

    end_pos = robot.solveForwardKinematics()[0]
    fx = [-10 * (end_pos[0] - 0.3), -10 * (end_pos[1] + 0.3), -10 * (end_pos[2] - 0.5)]

    #### Compute damping matrix ####
    e1 = np.array(fx)/np.linalg.norm(fx)
    e2_0 = np.array([1, 0, 0])
    e3_0 = np.array([0, 1, 0])
    e2 = e2_0 - np.dot(e2_0, (e1/np.linalg.norm(e1)))*(e1/np.linalg.norm(e1))
    e2 = e2/np.linalg.norm(e2)
    e3 = e3_0 - np.dot(e3_0, (e1/np.linalg.norm(e1)))*(e1/np.linalg.norm(e1)) - np.dot(e3_0, (e2 / np.linalg.norm(e2))) * (e2 / np.linalg.norm(e2))
    e3 = e3/np.linalg.norm(e3)
    Q = np.zeros([3, 3])
    Q[:, 0] = np.transpose(e1)
    Q[:, 1] = np.transpose(e2)
    Q[:, 2] = np.transpose(e3)

    Lambda = np.diag([lambda1, lambda2, lambda3])

    D = Q @ Lambda @ np.transpose(Q)

    xdot = robot.getEndVelocity()

    #### Compute target force ####
    fc = -D @ (np.transpose(np.array(xdot)) - np.transpose(fx))

    #### Compute target Wrench ####
    omega_current = robot.getEndAngularVelocity()

    ## Compute f_ori ##
    ori = robot.solveForwardKinematics()[1]
    # print(ori)

    """
    Orientation in a format of the scalar being the last number

    Essentially, logdq computes the angle-axis representation of the difference between
    two quaternions, or the rotation required to rotate from one orientation to another
    """

    ori_des = [1, 0, 0, 0]
    s1 = ori[3]
    s2 = ori_des[3]
    u1 = np.array([ori[0], ori[1], ori[2]])
    u2 = np.array([ori_des[0], ori_des[1], ori_des[2]])
    Su1 = np.array([[0, -u1[2], u1[1]], [u1[2], 0, -u1[0]], [-u1[1], u1[0], 0]])
    temp = -s1*u2 + s2*u1 - Su1@u2
    dori = np.array([s1*s2+u1@u2.T, temp[0], temp[1], temp[2]])
    logdq = np.arccos(dori[0])*(np.array([dori[1], dori[2], dori[3]]) / np.linalg.norm(np.array([dori[1], dori[2], dori[3]])))
    # omega_num = 2*logdq/stepsize
    f_ori = -10*logdq

    ## Compute fc_wrench ##
    e1 = np.array(f_ori)/np.linalg.norm(f_ori)
    e2_0 = np.array([1, 0, 0])
    e3_0 = np.array([0, 1, 0])
    e2 = e2_0 - np.dot(e2_0, (e1/np.linalg.norm(e1)))*(e1/np.linalg.norm(e1))
    e2 = e2/np.linalg.norm(e2)
    e3 = e3_0 - np.dot(e3_0, (e1/np.linalg.norm(e1)))*(e1/np.linalg.norm(e1)) - np.dot(e3_0, (e2 / np.linalg.norm(e2))) * (e2 / np.linalg.norm(e2))
    e3 = e3/np.linalg.norm(e3)
    Q = np.zeros([3, 3])
    Q[:, 0] = np.transpose(e1)
    Q[:, 1] = np.transpose(e2)
    Q[:, 2] = np.transpose(e3)

    Lambda = np.diag([1, 1, 1])

    D = Q @ Lambda @ np.transpose(Q)
    fc_wrench = -D @ (np.transpose(np.array(omega_current)) - np.transpose(f_ori))
    # fc_wrench = -10*np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) @ (np.array(omega_current) - logdq)
    # print(fc_wrench)

    #### Compute target torque ####
    Jacobian = np.zeros([6, 7])
    Jacobian[0, :] = np.array(robot.getJacobian()[0])
    Jacobian[1, :] = np.array(robot.getJacobian()[1])
    Jacobian[2, :] = np.array(robot.getJacobian()[2])
    Jacobian[3, :] = np.array(robot.getJacobian_ori()[0])
    Jacobian[4, :] = np.array(robot.getJacobian_ori()[1])
    Jacobian[5, :] = np.array(robot.getJacobian_ori()[2])
    Jacobian_pos = np.zeros([3, 7])
    Jacobian_pos[0, :] = np.array(robot.getJacobian()[0])
    Jacobian_pos[1, :] = np.array(robot.getJacobian()[1])
    Jacobian_pos[2, :] = np.array(robot.getJacobian()[2])
    # target_torque = [0, 0, 0, 0, 0, 0, 0]
    # target_torque =(np.linalg.pinv(Jacobian_pos)) @ fc
    target_torque = np.transpose(Jacobian) @ np.concatenate((fc, fc_wrench))
    # target_torque = np.linalg.pinv(Jacobian) @ np.concatenate((fc, fc_wrench))
    robot.setTargetTorques(target_torque)

    # robot.setTargetPositions(robot.solveInverseKinematics([0, 0.3, 0.3], [0, 0, 0, -1]))
    # joint_vel = np.linalg.pinv(Jacobian_pos) @ fx
    # joint_vel = np.linalg.pinv(Jacobian) @ np.concatenate((fx, f_ori))
    # print(robot.solveInverseKinematics([0, 0.3, 0.3], [1, 0, 0, 0]))
    # robot.setTargetVelocity(list(joint_vel))

    robot.step()
    if i % 50 == 0:
        # print(fc)
        print(robot.solveForwardKinematics()[1])
        # print(omega)
        # print(quat)
        # print("omega_lib: ", omega_current)
        # print(target_torque)
    time.sleep(robot.stepsize)