from src.robot_arm import RobotArm2D
from src.visualization import draw_robot
from src.visualization import draw_ik_solutions
import numpy as np
from src.plot_jacobian_det import plot_jacobian_det
from src.plot_end_effector_velocity import plot_end_effector_velocity
robot = RobotArm2D(2, 1)
draw_robot(robot)
draw_ik_solutions(robot, (0, 2))
draw_ik_solutions(robot, (3, 2))
draw_ik_solutions(robot, (2, 1))
plot_jacobian_det(robot)
plot_end_effector_velocity(robot, 0.5, 1.0)
