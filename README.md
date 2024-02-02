ROS Pybullet Interface









In python script, the main function will toggle the IK set up node which will then retrieve the current position and target position to solve the IK problem, as well as the figure 8 node that will continuosly publish the target positions and orientation to the topic. Hence, this topic receive targets from figure 8 node and provide the targets the IK problems. This is for Pybullet solving approach which defines the entire probelm without interface. 

Once the IK setup node receives all the information, the problem is then 'packed' or setup and the published. Once solved, the robot can move the desired target.


In previous example of moving to one position using IK is through a service rpbi/{robot_name}/move_to_eff_state. Here we are publishing the problem to a topic instead, rpbi/{robot_name}/ik. Nevertheless, they both take same message type - CalculateInverseKinematicsProblem.


For my implementation of going through a sequence of trajectory whether time stamped or not, the only difference that needs to be made is to change the desired trajectory in the figure 8 node with my own trajectory generated from the quat DS. However, this is only for now to visualize. Later on, need to use real control scheme to achieve so.


Pybullet seems promising, will look into traj_IK later and skip exotica. As mentioned in README, IK_traj is an alternative appraoch. 



The first to call that make intuitively sense is the rpbi node with its configuration, which exposes all the topics related to the robot simulation containing the ik, and joint states of the robot. We know the iniated node will publish the current joint states to the corresponding topic. In contrast, IK topic is also set up but this time, the inferface is subscribing to this topic, meaning that once the Inverse problem is packed and sent, the interface will then respond and move the robot.


To be more precise, Figure 8 node will publish the current targets to the /tf topic which contains the transformation of targets defined by frame id and child_frame_id both defined in the figure 8 node. Figure 8 node will not publish until its established toggle is turned on in the main Python script. Because all the motion targets are defined as child frame wrt the frame id, we need to define the frame ID and make sure it's published to the /tf as well, hence the second section in the launch file. The sole purpose of this node is to publish this transformation so it exists in the topic /tf.


When set up the IK setup node, the configuration requires the robot name, the link name of end effector, the parent and child frame similar to what we see before in publishing targets. The two topics exposed by the interface are both used and remapped in the setup. The setup node will subscribe to Joint states topic so it can retrieve the current information of the robot, and will publish the formulated problem to the IK topic so the robot can move, as well as subscribing to the figure 8 for the desired targets. 

The main Python script was mainly used to toggle all the nodes we just initiated and estabilished since they are not necessarily active until called. Such activation is done by trigger the toggle servies which are all defined and exposed in the respective node files. Moving to initial positiion - such a single one time task - can be efficiently done via calling the interface service. To toggle the services, sicne these services are already established and exposed, we can call them by establishing service handles and directly call them.







To-do:

1. Transfer all the code and clean up as a launch in ros_quat_ds package
2. Integrated the trajectory from quat_ds; take care of the time dependece
3. Explore ik_traj (Optional)