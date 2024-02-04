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
    file_path  = os.path.join(file_dir, "p_in.npy")


    position = np.load(file_path)[0].T 
    position /= 100


    

    return position


def load_traj(init_eff_pos, init_eff_rot):
    """ 
    Use to formulate the trajectory for run_interpolation_pybullet to load 

    Note: traj_plan has a shape of M by N where M consist of time; position; quaternion; position diff and quatenrion diff
    """

    total_time = 5 

    rospy.loginfo('generating plan by interpolating between waypoints...')

    rospy.loginfo('generating waypoints ...')


    init_pose = np.hstack((init_eff_pos, init_eff_rot))

    position = load_file()


    rotation = np.tile(init_eff_rot, (position.shape[0], 1))

    pos_body = np.vstack((init_pose, np.hstack((position, rotation)))).T



    diff = np.diff(pos_body, axis=1)
    velBody = pos_body*0.0
    scale_vel_factor = 0.5
    velBody[:3, 1:-1] = scale_vel_factor*(diff[:3, :-1] + diff[:3, 1:])/2.0



    timeSeq = np.linspace(0, total_time, pos_body.shape[1])

    traj_plan = np.vstack((np.vstack((timeSeq, pos_body)), velBody))


    rospy.loginfo(traj_plan.shape)

    return traj_plan