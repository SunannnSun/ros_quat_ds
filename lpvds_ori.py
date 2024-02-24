import os, sys, json
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats import multivariate_normal

from util.gmm import gmm as gmm_class
from util.quat_tools import *


"""
populate the file with functions and methods
"""


def compute_ang_vel(q_k, q_kp1, dt=0.1):
    """
    Compute angular velocity in q_k frame to rotate q_k into q_kp1 given known time difference
    """

    # dq = q_k.inv() * q_kp1


    dq = q_kp1.inv() * q_k


    dq = dq.as_rotvec()

    w  = 2 * dq / dt

    return w


class lpvds_ori:

    def __init__(self):
        """
        read parameters: clustering and optmization
        """

        js_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'output_ori.json')

        with open(js_path, 'r') as f:
            data = json.load(f)

        K = data['K']
        M = data['M']
        Priors = np.array(data['Priors'])
        Mu = np.array(data['Mu']).reshape(K, M)
        Sigma = np.array(data['Sigma']).reshape(K, M, M)

        att = np.array(data['att'])

        A = np.array(data['A']).reshape(K, 4, 4)
        
        q_normal_list = []

        for k in range(K):

            Mu_k = Mu[k, :]
            Sigma_k = Sigma[k, :, :]

            q_normal_list.append(
                {
                    "mu": R.from_quat(Mu_k),
                    "sigma": Sigma_k,
                    "rv": multivariate_normal(np.zeros((4, )), Sigma_k, allow_singular=True)
                }
            )

        gmm = gmm_class(q_in=[], q_att=R.from_quat(att))
        gmm.K = K
        gmm.Prior = Priors
        gmm.q_normal_list = q_normal_list

        self.K = K
        self.A = A
        self.q_att = R.from_quat(att)
        self.gmm = gmm

        self.dual_gmm = gmm._dual_gmm()
        

    def step(self, q_in, dt):

        K = self.K
        A = self.A
        q_att = self.q_att
        gmm   = self.gmm

        dual_gmm = self.dual_gmm
        w_init = dual_gmm.postLogProb(q_in).T

        index_of_largest = np.argmax(w_init)

        if index_of_largest <= (dual_gmm.K/2 - 1):
            pass
        else:
            q_in = R.from_quat(-q_in.as_quat())


        q_in_att  = riem_log(q_att, q_in)
        q_out_att = np.zeros((4, 1))

        w_k = gmm.postLogProb(q_in)
        for k in range(K):
            q_out_att += w_k[k, 0] * A[k] @ q_in_att.reshape(-1, 1)

        q_out_body = parallel_transport(q_att, q_in, q_out_att.T)
        q_out_q    = riem_exp(q_in, q_out_body) 
        q_out      = R.from_quat(q_out_q.reshape(4,))

        w_out      = compute_ang_vel(q_in, q_out, dt)
        # q_next     = q_in * R.from_rotvec(w_out * dt)

        # return q_next, w_k

        return w_out, w_k