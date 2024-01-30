#!/usr/bin/env python3
import sys
import math
import time
import rospy
import numpy as np
from rpbi.ros_node import RosNode

from std_msgs.msg import Int64
from sensor_msgs.msg import JointState

from ros_pybullet_interface.msg import CalculateInverseKinematicsProblem
from ros_pybullet_interface.srv import RobotInfo
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateRequest
from ros_pybullet_interface.srv import ResetEffState, ResetEffStateRequest

import tf_conversions

from custom_ros_tools.tf import TfInterface
from custom_ros_tools.ros_comm import get_srv_handler


class TfTracker:

    def __init__(self, tf, parent, child):
        self.tf = tf
        self.parent = parent
        self.child = child
        rospy.Timer(rospy.Duration(0.01), self.main_loop)
        # self.transform = None

    def main_loop(self, event):
        self.transform = self.tf.get_tf_msg(self.parent, self.child)

    def distance(self):
        if self.transform is None:
            return np.inf
        return np.linalg.norm(self.tf.msg_to_pos(self.transform))

    def get_pos(self):
        return self.tf.msg_to_pos(self.transform)
    


class Node(RosNode):

    hz = 50
    dt = 1.0/float(hz)

    def __init__(self):
        rospy.init_node('run_franka_eff_state')
        
        self.tf = TfInterface()
        self.eff = TfTracker(self.tf, 'rpbi/world', 'rpbi/franka/panda_link8')


        if rospy.has_param('~robot_name'):
            robot_name = rospy.get_param('~robot_name')
        elif len(sys.argv) > 1:
            robot_name = sys.argv[1]
            rospy.loginfo('Robot Name: %s' % robot_name)
        else:
            raise ValueError("robot name not specified, either set the '~robot_name' node parameter or via command line args")
        

        self.move_to_eff_state = get_srv_handler(f'rpbi/{robot_name}/move_to_eff_state', ResetEffState, persistent=True)
      
        self.move_to_initial_pose()

        # self.move_to_given_pose()
        self.move_to_initial_pose()

        # rospy.loginfo(self.eff.get_pos())


    def move_to_given_pose(self):
        """
        Unlike move_to_initial_pose which moves to a pre-defined initial state defined in the launch file, this method
        can move to any given pose which consists of a target position and target orientation. Such target can either be
        defined wrt the world frame or any frame in general.
        """
        rospy.loginfo('moving robot to given state: ')
        
        self.tf.set_tf('rpbi/world',
                        'target',
                        0.2* np.ones((3,)),
                        tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        )
        """
        It takes time to broadcast the transformation. Hence if we try to get the updated or the new transformation immediately
        after broadcasting the new transformation, it might fail to retreive the updated information. Hence, some latency used
        to ensure the broadcasting is done.
        """
        
        timeout = 5
        start_time = time.time()

        while (time.time()-start_time) < timeout:
            init_eff_pos, init_eff_rot = self.tf.get_tf('rpbi/world', 'target')
            if init_eff_pos is not None:
                rospy.loginfo(init_eff_pos)
                break
        else:
            rospy.logerr("could not recieve end-effector target transform")
            sys.exit(0)


        problem = CalculateInverseKinematicsProblem()
        problem.link_name = 'panda_link8'
        problem.targetPosition = init_eff_pos
        problem.targetOrientation = init_eff_rot

        duration = 2.0
        req = ResetEffStateRequest(problem=problem, duration=duration)

        self.move_to_eff_state(req)


    def move_to_initial_pose(self):
        """
        From my understanding, self.tf.get_tf('rpbi/world', 'origin') is to fetch the transformation of the second
        argument wrt the first argument. Since 'origin' is defined in the launch file wrt the 'rpi/world', we can 
        retrieve the correct transformation which returns a tuple of position and rotation (refer to msg file for 
        details and types). 

        Note:
        'rpbi/world' is the frame_id not the node name, so is the 'origin' while 'origin_tf' is the node name. 
        """
        rospy.loginfo('moving robot to start state: ')
        
        # Get eff target
        timeout = 5.0
        start_time = time.time()
        while (time.time()-start_time) < timeout:
            init_eff_pos, init_eff_rot = self.tf.get_tf('rpbi/world', 'origin')
            if init_eff_pos is not None:
                rospy.loginfo(init_eff_pos)
                break
        else:
            rospy.logerr("could not recieve end-effector target transform")
            sys.exit(0)

        # Setup problem
        problem = CalculateInverseKinematicsProblem()
        problem.link_name = 'panda_link8'
        problem.targetPosition = init_eff_pos
        problem.targetOrientation = init_eff_rot

        # Setup request
        duration = 2.0
        req = ResetEffStateRequest(problem=problem, duration=duration)

        # Move robot
        self.move_to_eff_state(req)


    def spin(self):
        rospy.Rate(100.0)
        rospy.spin()


def main():
    Node().spin()


if __name__=='__main__':
    main()
