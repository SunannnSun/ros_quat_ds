Pybullet of Franka Panda, modified version of Zhiquan's implementation


To-do:

1. Move to initial position: required initial position as an input


Take note on Priors and Prior ... seems like only Priors is used.


Need to verify to which frame is the rotational vector with respect: it's likely that the default computation computes rotational representation in world frame while my quat_ds computes angular veclotiy in body frame


Essentially, what's being used in simulation/lpv_ds function requires different sizes on parameters than the one being stored in json file for the actual execution - this needs to be taken care of later when implementing in ROS simulation but okay for now.


Read DAMM parameters (Priors, Mu, Sigma) from the post-DAMM logout JSON file, and read DS parameters from post-DS_OPT results while prior logout JSON file.


Both DAMM and DS_OPT parameters from the post logout JSON file are only used for real robot or potential ROS simulation.