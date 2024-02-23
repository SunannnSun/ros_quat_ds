
import numpy as np

from lpvds_pos import lpvds_pos as lpvds_pos_class
from lpvds_ori import lpvds_ori as lpvds_ori_class

from robot_sim import robot_sim as robot_sim_class



init_pos = np.array([0.47143134, -0.20502549, 0.21919133])
init_ori = np.array([0.70023204, -0.00707885, -0.03352073,  0.7130928 ])


lpvds_pos = lpvds_pos_class()
lpvds_ori = lpvds_ori_class()
sim = robot_sim_class()

sim.move_to(init_pos, init_ori)

sim.execute_ds(lpvds_pos, lpvds_ori)