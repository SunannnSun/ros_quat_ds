ROS Pybullet Interface









In python script, the main function will toggle the IK set up node which will then retrieve the current position and target position to solve the IK problem, as well
as the figure 8 node that will continuosly publish the target positions and orientation to the topic. Hence, this topic receive targets from figure 8 node and provide
the targets the IK problems. This is for Pybullet solving approach which defines the entire probelm without interface. 

Once the IK setup node receives all the information, the problem is then 'packed' or setup and the published. Once solved, the robot can move the desired target.


In previous example of moving to one position using IK is through a service rpbi/{robot_name}/move_to_eff_state. Here we are publishing the problem to a topic instead,
rpbi/{robot_name}/ik. Nevertheless, they both take same message type - CalculateInverseKinematicsProblem.


For my implementation of going through a sequence of trajectory whether time stamped or not, the only difference that needs to be made is to change the desired trajectory
in the figure 8 node with my own trajectory generated from the quat DS. However, this is only for now to visualize. Later on, need to use real control scheme to achieve so.


Pybullet seems promising, will look into traj_IK later and skip exotica. As mentioned in README, IK_traj is an alternative appraoch. 