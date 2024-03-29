import sys
import numpy as np

from src.panda_noball import Panda
from scipy.spatial.transform import Rotation as R




class robot_sim:

    def __init__(self):

        self.stepsize = 1e-3
        self.duration = 50
        self.tol      = 8e-3
        self.max_iter = int(self.duration / self.stepsize)

        self.robot = Panda(self.stepsize)
        self.robot.setControlMode("torque")

        self.lambda1 = 20
        self.lambda2 = 100
        self.lambda3 = 100


    def move_to(self, end_pos, end_ori=[1, 0, 0, 0]):
        robot = self.robot
        curr_pos = robot.solveForwardKinematics()[0]
        curr_ori = robot.solveForwardKinematics()[1]

        i = 0
        while np.linalg.norm(curr_pos - end_pos) > self.tol:
            if i > self.max_iter:
                print("Exceed max iteration")
                sys.exit(1)
            if i % 10000 == 0:
                print("Simulation time: {:.3f}".format(robot.t))
            
            fx = -1 * (curr_pos - end_pos)
            f_ori = compute_f_ori(curr_ori, end_ori)
            curr_pos, curr_ori = self._step(fx, f_ori)

        print("Reached the target pose")


    def execute_ds(self, lpvds_pos, lpvds_ori):
        robot = self.robot
        curr_pos = np.array(robot.solveForwardKinematics()[0])
        curr_ori = np.array(robot.solveForwardKinematics()[1])

        end_pos = lpvds_pos.att
        end_ori = lpvds_ori.q_att

        i = 0
        w_k = np.array([1, 0, 0, 0])
        while np.linalg.norm(curr_pos-end_pos) > self.tol or np.linalg.norm(curr_ori-end_ori.as_quat()) > self.tol:
            if i > self.max_iter:
                print("Exceed max iteration")
                sys.exit(1)
            if i % 10000 == 0:
                print("Simulation time: {:.3f}".format(robot.t))
                print("Distance to pos_att: {:.3f}".format(np.linalg.norm(end_pos-curr_pos)))
                print("Distance to ori_att: {:.3f}".format(np.linalg.norm(curr_ori-end_ori.as_quat())))
                print("Quat DS index:   {:d}".format(np.argmax(w_k)))


            fx = lpvds_pos.step(curr_pos)[:, 0]

            f_ori, w_k = lpvds_ori.step(curr_pos.reshape(1, -1), R.from_quat(curr_ori), self.stepsize)

            xdot = robot.getEndVelocity()

            # if np.linalg.norm(xdot) < 10E-2 and np.linalg.norm(curr_pos-end_pos) > self.tol:
            #     f_ori *= np.linalg.norm(xdot)


            curr_pos, curr_ori = self._step(3 * fx, 1/10 * f_ori)

        print("Reached the target pose")


    def _step(self, fx, f_ori):
        """
        Step forward using fx and f_ori and return the ee position
        """
        robot = self.robot

        # curr_pos = robot.solveForwardKinematics()[0]
        # curr_pos = np.array(curr_pos)
        # fx = -2 * (curr_pos - end_pos)

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

        Lambda = np.diag([self.lambda1, self.lambda2, self.lambda3])

        D = Q @ Lambda @ np.transpose(Q)

        xdot = robot.getEndVelocity()

        #### Compute target force ####
        fc = -D @ (np.transpose(np.array(xdot)) - np.transpose(fx))

        #### Compute target Wrench ####
        omega_current = robot.getEndAngularVelocity()

        ## Compute f_ori ##
        # ori = robot.solveForwardKinematics()[1]

        # """
        # Orientation in a format of the scalar being the last number

        # Essentially, logdq computes the angle-axis representation of the difference between
        # two quaternions, or the rotation required to rotate from one orientation to another
        # """

        ## Compute fc_wrench ##
        e1 = f_ori/np.linalg.norm(f_ori)
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

        target_torque = np.transpose(Jacobian) @ np.concatenate((fc, fc_wrench))

        robot.setTargetTorques(target_torque)

        robot.step()

        return np.array(robot.solveForwardKinematics()[0]), np.array(robot.solveForwardKinematics()[1])
    



def compute_f_ori(curr_ori, end_ori=[1, 0, 0, 0]):
    s1 = curr_ori[3]
    s2 = end_ori[3]
    u1 = np.array([curr_ori[0], curr_ori[1], curr_ori[2]])
    u2 = np.array([end_ori[0], end_ori[1], end_ori[2]])
    Su1 = np.array([[0, -u1[2], u1[1]], [u1[2], 0, -u1[0]], [-u1[1], u1[0], 0]])
    temp = -s1*u2 + s2*u1 - Su1@u2
    dori = np.array([s1*s2+u1@u2.T, temp[0], temp[1], temp[2]])
    logdq = np.arccos(dori[0])*(np.array([dori[1], dori[2], dori[3]]) / np.linalg.norm(np.array([dori[1], dori[2], dori[3]])))
    f_ori = -10*logdq

    return f_ori