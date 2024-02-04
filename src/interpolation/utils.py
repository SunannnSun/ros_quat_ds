#!/usr/bin/env python3
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt

from custom_ros_tools.tf import TfInterface
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension



def load_traj(init_eff_pos, init_eff_rot):
    """ 
    Use to formulate the trajectory for run_interpolation_pybullet to load 

    Note: traj_plan has a shape of M by N where M consist of time; position; quaternion; position diff and quatenrion diff
    """


    rospy.loginfo('generating plan by interpolating between waypoints...')

    rospy.loginfo('generating waypoints ...')


    timeSeq = np.array([0.0, 4.0, 9.0, 14.0])

    init_pose = np.hstack((init_eff_pos, init_eff_rot))

    wpt1_pos = np.array([0.6, 0.1, 0.3])
    wpt1 = np.hstack((wpt1_pos, init_eff_rot))

    wpt2_pos = np.array([0.4, 0.4, 0.3])
    wpt2 = np.hstack((wpt2_pos, init_eff_rot))

    wpt3_pos = np.array([0.1, 0.6, 0.2])
    wpt3 = np.hstack((wpt3_pos, init_eff_rot))

    pos_body = np.vstack((init_pose, wpt1))
    pos_body = np.vstack((pos_body, wpt2))
    pos_body = np.vstack((pos_body, wpt3)).T

    diff = np.diff(pos_body, axis=1)
    velBody = pos_body*0.0
    scale_vel_factor = 0.5
    velBody[:3, 1:-1] = scale_vel_factor*(diff[:3, :-1] + diff[:3, 1:])/2.0

    traj_plan = np.vstack((np.vstack((timeSeq, pos_body)), velBody))


    rospy.loginfo(traj_plan.shape)

    return traj_plan