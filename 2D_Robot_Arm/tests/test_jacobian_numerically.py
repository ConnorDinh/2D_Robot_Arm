from src.robot_arm import RobotArm2D
from src.jacobian import end_effector_velocity
import numpy as np

def test_jacobian_numerically():
    robot = RobotArm2D(2, 1)
    theta1 = robot.convert_degree_to_radiant(30)
    theta2 = robot.convert_degree_to_radiant(45)
    robot.set_joint_angles(theta1, theta2)
    theta1_dot = 0.5
    theta2_dot = 1.0
    dt = 0.00001
    old_position = robot.forward_kinematics()
    jacobian_velocity = end_effector_velocity(robot, theta1_dot, theta2_dot)
    theta1_new = theta1 + theta1_dot * dt
    theta2_new = theta2 + theta2_dot * dt
    robot.set_joint_angles(theta1_new, theta2_new)
    new_position = robot.forward_kinematics()
    numerical_velocity = np.array([(new_position[0] - old_position[0]) / dt, (new_position[1] - old_position[1]) / dt])
    np.testing.assert_allclose(numerical_velocity, jacobian_velocity, rtol=1e-4, atol=1e-4)

test_jacobian_numerically()

