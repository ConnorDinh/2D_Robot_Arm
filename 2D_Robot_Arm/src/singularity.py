import numpy as np

def is_singular(robot, tolerance):
    theta2 = robot.theta2
    det = robot.L1 * robot.L2 * np.sin(theta2)
    if abs(det) < tolerance:
        return True
    return False