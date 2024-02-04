#!/usr/bin/env python3
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt

from custom_ros_tools.tf import TfInterface
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

from interpolation.utils import load_traj


class Node:

    def __init__(self):
        rospy.init_node('run_interpolation_node')
        self.tf = TfInterface()

        # initialize empty trajectory plan
        self.traj_plan = np.empty(0)

        # topic where sequence of waypoints is published
        wpts_topic_name = rospy.get_param(
            '~topic_name_4_waypoints', 'ros_pybullet_interface/waypt_traj')

        # init the publisher
        self.new_traj_publisher = rospy.Publisher(
            f"{wpts_topic_name}", Float64MultiArray, queue_size=1)

        # create two lists for storing and plotting reasons
        self.actual_motion = []
        self.interpol_plan = []
        self.time_of_motion = []
        self.init_time = 0.0

    def publish_trajectory(self, event):

        message = self.np2D_to_ROSmsg(self.traj_plan)

        if message is not None:
            self.new_traj_publisher.publish(message)

    def np2D_to_ROSmsg(self, data2D_array):

        # check if there is nothing to publish
        if data2D_array.size == 0:
            return

        r, c = data2D_array.shape

        # info: http://docs.ros.org/en/api/std_msgs/html/msg/MultiArrayLayout.html
        # Pack trajectory msg
        msg = Float64MultiArray()

        # specify that the array has 2 dimensions
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())

        # info for reconstruction of the 2D array
        msg.layout.dim[0].label = "rows"
        msg.layout.dim[0].size = r
        msg.layout.dim[1].label = "columns"
        msg.layout.dim[1].size = c

        # add data as flattened numpy array
        msg.data = data2D_array.flatten('C')  # row major flattening

        # time is missing from this message
        # msg.header.stamp = rospy.Time.now()
        return msg

    def exec_plan(self):
         
        #  get current TF of robot end effector
        init_eff_pos, init_eff_rot = self.tf.wait_for_tf(
            'rpbi/world', 'rpbi/franka/panda_link8')
    
        self.traj_plan = load_traj(init_eff_pos, init_eff_rot)

        # publish the waypoint plan
        self.write_callback_timer = rospy.Timer(
            rospy.Duration(1.0/float(2)), self.publish_trajectory)

        rospy.loginfo("Waypoints were published!")



    def collect_interpol_plan_and_actual_motion_data(self):

        interpol_eff_pos, _ = self.tf.get_tf('rpbi/franka/panda_link0', 'interpol_target')
        curr_eff_pos, _ = self.tf.get_tf('rpbi/world', 'rpbi/franka/panda_link8')

        if interpol_eff_pos is not None:
            self.interpol_plan.append(interpol_eff_pos)
            self.actual_motion.append(curr_eff_pos)
            self.time_of_motion.append(rospy.Time.now().to_sec() - self.init_time)
        else:
            self.init_time = rospy.Time.now().to_sec()

    def plot_results(self):

        #
        interpol_plan = np.asarray(self.interpol_plan)
        actual_motion = np.asarray(self.actual_motion)

        time_of_motion = self.time_of_motion

        fig = plt.figure()
        ax1 = plt.subplot(1, 3, 1)
        ax1.plot(self.traj_plan[0, :], self.traj_plan[1, :], '*', markersize=14, label='Waypts')
        ax1.plot(time_of_motion, interpol_plan[:, 0], 'b', label='Inpterpolated plan')
        ax1.plot(time_of_motion, actual_motion[:, 0], 'r', label='Actual motion')
        ax1.legend(loc="upper left")
        ax1.set_title("X axis")

        ax2 = plt.subplot(1, 3, 2)
        ax2.plot(self.traj_plan[0, :], self.traj_plan[2, :], '*', markersize=14)
        ax2.plot(time_of_motion, interpol_plan[:, 1], 'b')
        ax2.plot(time_of_motion, actual_motion[:, 1], 'r')
        ax2.set_title("Y axis")

        ax3 = plt.subplot(1, 3, 3)
        ax3.plot(self.traj_plan[0, :], self.traj_plan[3, :], '*', markersize=14)
        ax3.plot(time_of_motion, interpol_plan[:, 2], 'b')
        ax3.plot(time_of_motion, actual_motion[:, 2], 'r')
        ax3.set_title("Z axis")

        fig.suptitle("Linear axes interpolation results", fontsize=16)
        plt.show()

    def spin(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            self.collect_interpol_plan_and_actual_motion_data()
            rate.sleep()
        self.plot_results()


def main():
    node = Node()
    node.exec_plan()
    node.spin()


if __name__ == '__main__':
    main()
