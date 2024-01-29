#!/usr/bin/env python3
import sys
import math
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from ros_pybullet_interface.srv import RobotInfo
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateRequest
from custom_ros_tools.ros_comm import get_srv_handler

from rpbi.ros_node import RosNode


class Node(RosNode):

    hz = 50
    dt = 1.0/float(hz)

    def __init__(self):

        # Init node
        rospy.init_node('run_franka_joint_state')

        """
        The robot_name in this case is not defined in the parameter server. Hence, the second condition
        is called when the argument passed in the launch file gives the robot name to be specified
        """
        if rospy.has_param('~robot_name'):
            robot_name = rospy.get_param('~robot_name')
        elif len(sys.argv) > 1:
            robot_name = sys.argv[1]
            rospy.loginfo('Robot Name: %s' % robot_name)
        else:
            raise ValueError("robot name not specified, either set the '~robot_name' node parameter or via command line args")
        


        """
        rpbi/status is a topic that is defined and available when initiating RPBI, and it should always be True when RPBI is on.
        Since the entire program is reliant on RBPI, this line ensures that we subscribe to the statue of RPBI and ensure its ON.
        """
        self.active = False
        rospy.Subscriber('rpbi/status', Int64, self.active_callback)



        """
        When calling get_srv_handler, the service as named in the string argument must exist
        in the rosservice list. Now we have the handle to an existing service, we can call the
        service by passing arugments or directly calling. Once called, the handle will return the
        response. We can look into the .srv file to confirm the data and structure of the return
        """
        handle = get_srv_handler(f'rpbi/{robot_name}/robot_info', RobotInfo)
        res = handle()
        self.ndof = res.numDof
        self.name = [j.jointName for j in res.joint_info if j.jointTypeStr != 'JOINT_FIXED']

        rospy.loginfo('numDof: %f' % res.numDof)
        rospy.loginfo('jointName: %s' % self.name[0])

        
        """
        The previous server returned and constructed a name list with each entry the name of joint that
        is specified in urdf file. The lines below construct an initial state (in radians) for each joint, 
        as well as joint trajectory that is identical for each joint.
        """
        self.joint_index = 0
        self.position = [0.0*math.pi/180.0]*self.ndof 
        self.d = 1  ### d is the increment of joints when one joint is done with its traj; in this case we are moving up by d=1
        self.traj_index = 0
        self.joint_traj = [math.sin(0.5*2.0*math.pi*float(i)/100.0) for i in range(200)]



        """
        Similarly, before defining the handler to a service, ensure the service is indeed defined and available.
        The second argument of the handle to the service move_to_joint_state, the ResetJointState, is defined 
        to pass by arguments, a target joint state and a time duration. Once the specified state is reach, one
        can notice a string message printed which is indeed the return of the service as in .srv file

        Note: the JointState is already defined, and we can create a such instance by looking at the example in
        method: get_goal_joint_state, which is to call JointState(name=self.name, position=self.position) so that
        a mapping is available from a list of names to a list of joint states 
        """
        rospy.loginfo('moving robot to initial joint state')
        duration = 3.0
        handle = get_srv_handler(f'rpbi/{robot_name}/move_to_joint_state', ResetJointState)
        handle(self.get_goal_joint_state(), duration)


        """
        To continuously publish or control the joint states to follow the pre-defined trajectory, we first define a 
        topic and initiate a publisher. Similar to service, when initiating a publisher, the first arugment is the 
        topic name and we have to ensure it exists in the rostopic list first, and the second arugment defines the 
        message type we are publishing to the topic, in this case, we are publishing JointState same as the arugment
        we previously pass to the move_to_joint_state service.
        """
        self.pub = rospy.Publisher(f'rpbi/{robot_name}/joint_states/target', JointState, queue_size=10)


        """
        Up now, the publisher is established, but nothing has been published. To continously update the joint states 
        to follow the trajectory, we call the timer to invoke the defined function at a given duration
        """
        dur = rospy.Duration(self.dt)
        self.Timer(dur, self.publish_joint_state)




    def active_callback(self, msg):
        self.active = bool(msg.data)

    def update_joint_index(self):
        self.joint_index += self.d
        if self.joint_index == self.ndof:
            self.joint_index -= 2
            self.d = -1
        if self.joint_index == -1:
            self.joint_index = 0
            self.d = 1

    def update_trajectory_index(self):
        self.traj_index += 1
        if self.traj_index == len(self.joint_traj):
            self.traj_index = 0
            self.position = [0.0]*self.ndof  ### This line returns the robot to initial state every time a joint is done with its trajectory
            self.update_joint_index()

    def get_goal_joint_state(self):
        return JointState(name=self.name, position=self.position)

    def publish_joint_state(self, event):
        """
        When we have a name and joint state ready, we pass to self.get_goal_joint_state(), and use
        the returned JointState type message as the argument (agrees with the type as in initialization) 
        to publish to the topic /joint_states/target. So the system will drive from the current state
        to the defined target state
        """
        if not self.active: return     # If self.active=False, immediately returns nothing and leaves

        self.position[self.joint_index] = self.joint_traj[self.traj_index]
        self.pub.publish(self.get_goal_joint_state())
        self.update_trajectory_index()

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__=='__main__':
    main()
