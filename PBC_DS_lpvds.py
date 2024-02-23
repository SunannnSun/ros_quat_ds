
import numpy as np

from lpv_ds import lpv_ds_class
from robot_sim import robot_sim as robot_sim_class



init_pos = np.array([0.47143134, -0.20502549, 0.21919133])


lpv_ds = lpv_ds_class()
sim = robot_sim_class()

sim.move_to(init_pos, 0)

sim.execute_ds(lpv_ds)