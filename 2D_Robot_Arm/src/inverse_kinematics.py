import numpy as np

def analytical_ik(robot, x, y):
    if not robot.is_reachable(x, y):
        return
    L1 = robot.L1
    L2 = robot.L2
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2)/(2*L1*L2)
    cos_theta2 = max(-1, cos_theta2)
    cos_theta2 = min(1, cos_theta2)
    theta2s = []
    theta2s.append(np.arccos(cos_theta2))
    theta2s.append(-np.arccos(cos_theta2))
    solutions = []
    for theta2 in theta2s:
        gamma = np.atan2(y, x)
        beta = np.atan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
        theta1 = gamma - beta
        solutions.append((theta1, theta2))
    return solutions