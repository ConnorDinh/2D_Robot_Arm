from src.jacobian import compute_jacobian
import numpy as np

def compute_dls_update(robot, error, alpha, damping):
    J = compute_jacobian(robot)
    JT = np.transpose(J)
    I = np.identity(J.shape[0])
    A = J @ JT + damping**2 * I
    y = np.linalg.solve(A, np.array(error))
    delta_theta = alpha * (JT @ y)
    return delta_theta

def damped_numerical_ik(robot, target_x, target_y, alpha, damping, tolerance, max_iterations, history):
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
        delta_theta = compute_dls_update(robot, error, alpha, damping)
        theta1_new = robot.theta1 + delta_theta[0]
        theta2_new = robot.theta2 + delta_theta[1]
        robot.set_joint_angles(theta1_new, theta2_new)
        if history is not None:
            history.append((theta1_new, theta2_new))
    return False