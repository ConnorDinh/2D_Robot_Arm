import numpy as np

def compute_jacobian(robot):
    L1 = robot.L1
    L2 = robot.L2
    theta1 = robot.theta1
    theta2 = robot.theta2

    J11 = -L1*np.sin(theta1) - L2*np.sin(theta1 + theta2)
    J12 = -L2*np.sin(theta1 + theta2)
    J21 = L1*np.cos(theta1) + L2*np.cos(theta1 + theta2)
    J22 = L2*np.cos(theta1 + theta2)
    jacobian = np.array([[J11, J12], [J21, J22]])
    return jacobian

def end_effector_velocity(robot, theta1_dot, theta2_dot):
    J = compute_jacobian(robot)
    joint_velocities = np.array([theta1_dot, theta2_dot])
    velocity = J @ joint_velocities
    return velocity
