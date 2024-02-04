#!/usr/bin/env python3
import sys
import time
import rospy
from custom_ros_tools.tf import TfInterface
from geometry_msgs.msg import Transform
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState


from ros_pybullet_interface.msg import CalculateInverseKinematicsProblem
from ros_pybullet_interface.srv import ResetEffState, ResetEffStateRequest
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateRequest
from custom_ros_tools.ros_comm import get_srv_handler

class Node:

    move_to_start_duration = 5.0

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('run_follow_traj')

        # Setup tf interface
        self.tf = TfInterface()

        # Get parameters
        self.interface_name = sys.argv[1]
    

        self.robot_name = rospy.get_param(f'/ik_setup/config/robot_name')

        self.eff_name = rospy.get_param(f'/ik_setup/config/link_name')


        # Get service handles
        self.move_to_eff_state = get_srv_handler(f'rpbi/{self.robot_name}/move_to_eff_state', ResetEffState)
        self.move_to_joint_state = get_srv_handler(f'rpbi/{self.robot_name}/move_to_joint_state', ResetJointState)


        self.start_traj_setup_node = get_srv_handler('toggle_traj_setup', SetBool)
        self.start_ik_setup_node = get_srv_handler(f'ik/setup/{self.interface_name}/toggle', SetBool)


        self.move_to_start_pose = self.move_to_start_pose_using_pybullet
        self.solve_ik = None
        

        # Handle real robot
        self.toggle_remapper = None
        self.real_robot = rospy.get_param('real_robot', False)
        if self.real_robot:
            self.toggle_remapper = get_srv_handler('remap_joint_state_to_floatarray/toggle', SetBool)
            self.sync_pybullet_with_real_robot()


    def sync_pybullet_with_real_robot(self):
        duration = 0.1
        self.move_to_joint_state(rospy.wait_for_message('joint_states', JointState), duration)
        rospy.sleep(2)


    def get_start_pose(self):

        timeout = 10.0
        start_time = time.time()
        while (time.time()-start_time) < timeout:
            init_eff_pos, init_eff_rot = self.tf.get_tf('rpbi/world', 'traj_base')
            if init_eff_pos is not None:
                break
        else:
            rospy.logerr("could not recieve end-effector target transform")
            sys.exit(0)

        return init_eff_pos, init_eff_rot
    

    def move_to_start_pose_using_pybullet(self):

        init_eff_pos, init_eff_rot = self.get_start_pose()


        # import numpy as np
        # init_eff_pos = np.array([0.4, 0.4, 0.17])
        # init_eff_rot = np.array([9.99999683e-01, 0.00000000e+00, 0.00000000e+00, 7.96326711e-04])


        # Setup problem
        problem = CalculateInverseKinematicsProblem()
        problem.link_name = self.eff_name
        problem.targetPosition = init_eff_pos
        problem.targetOrientation = init_eff_rot

        # Setup request
        duration = self.move_to_start_duration
        req = ResetEffStateRequest(problem=problem, duration=duration)

        # Start remapper
        if self.real_robot:
            self.toggle_remapper(True)

        # Move robot
        self.move_to_eff_state(req)




    def start_motion(self):
        self.start_ik_setup_node(True)
        self.start_traj_setup_node(True)


def main():
    node = Node()
    node.move_to_start_pose()
    node.start_motion()


if __name__ == '__main__':
    main()
