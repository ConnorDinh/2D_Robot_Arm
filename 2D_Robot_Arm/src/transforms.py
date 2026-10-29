import numpy as np

def rotation_matrix(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])

def homogeneous_transform(theta, length):
    R = rotation_matrix(theta)
    translation_x = length*np.cos(theta)
    translation_y = length*np.sin(theta)
    return np.array([[R[0][0], R[0][1], translation_x], [R[1][0], R[1][1], translation_y], [0, 0, 1]])
