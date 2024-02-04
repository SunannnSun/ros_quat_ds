#!/usr/bin/env python3
import sys, os 
import rospy
import numpy as np
import matplotlib.pyplot as plt

from custom_ros_tools.tf import TfInterface
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


def load_file():

    file_dir = os.path.dirname(os.path.realpath(__file__)) 

    p_path  = os.path.join(file_dir, "p_in.npy")

    position = np.load(p_path)[0].T 
    position /= 100

    q_path  = os.path.join(file_dir, "q_in.npy")

    rotation_raw = np.load(q_path, allow_pickle=True)
    rotation =  np.zeros((rotation_raw.shape[0], 4))

    w      = rotation_raw[:, -1].copy()
    xyz    = rotation_raw[:, :3].copy()

    rotation[:, 0]  = w
    rotation[:, 1:] = xyz

    return position, rotation


def load_traj():
    """ 
    Use to formulate the trajectory for run_interpolation_pybullet to load 

    Note: traj_plan has a shape of M by N where M consist of time; position; quaternion; position diff and quatenrion diff
    """

    total_time = 15 

    rospy.loginfo('generating plan by interpolating between waypoints...')

    rospy.loginfo('generating waypoints ...')

    position, rotation = load_file()

    # position = np.tile(np.array([0.6, 0, 0.3]), (rotation.shape[0], 1))


    # rotation = np.tile(np.array([1, 0, 0, 0]), (position.shape[0], 1))

    pos_body = np.hstack((position, rotation)).T



    diff = np.diff(pos_body, axis=1)
    velBody = pos_body*0.0
    scale_vel_factor = 0.5
    velBody[:3, 1:-1] = scale_vel_factor*(diff[:3, :-1] + diff[:3, 1:])/2.0



    timeSeq = np.linspace(0, total_time, pos_body.shape[1])

    traj_plan = np.vstack((np.vstack((timeSeq, pos_body)), velBody))


    rospy.loginfo(traj_plan.shape)

    return traj_plan