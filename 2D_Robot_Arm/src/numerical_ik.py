import numpy as np
from src.jacobian import compute_jacobian

def compute_pseudoinverse(robot):
    J = compute_jacobian(robot)
    return np.linalg.pinv(J)

def numerical_ik(robot, target_x, target_y, alpha, tolerance, max_iterations, history=None):
    if not robot.is_reachable(target_x, target_y):
        return False
    if history is not None:
        history.append((robot.theta1, robot.theta2))
    for i in range(max_iterations):
        current = robot.forward_kinematics()
        error = np.array([target_x - current[0], target_y - current[1]])
        error_norm = np.linalg.norm(error)
        if error_norm < tolerance:
            return True
        J_pinv = compute_pseudoinverse(robot)
        delta_theta = alpha * (J_pinv @ np.array(error))
        theta1_new = robot.theta1 + delta_theta[0]
        theta2_new = robot.theta2 + delta_theta[1]
        robot.set_joint_angles(theta1_new, theta2_new)
        if history is not None:
            history.append((theta1_new, theta2_new))
    return False
