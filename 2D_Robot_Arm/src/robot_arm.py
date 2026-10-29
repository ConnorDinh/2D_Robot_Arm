import numpy as np
import math
from src.transforms import homogeneous_transform

class RobotArm2D:
    def __init__(self, link1, link2):
        self.L1 = link1
        self.L2 = link2
        self.theta1 = 0
        self.theta2 = 0

    def set_joint_angles(self, theta1, theta2):
        self.theta1 = theta1
        self.theta2 = theta2

    def get_joint_positions(self):
        base = (0, 0)

        joint1_x = self.L1 * np.cos(self.theta1)
        joint1_y = self.L1 * np.sin(self.theta1)

        joint2_x = joint1_x + self.L2 * np.cos(self.theta1 + self.theta2)
        joint2_y = joint1_y + self.L2 * np.sin(self.theta1 + self.theta2)

        return base, (joint1_x, joint1_y), (joint2_x, joint2_y)

    def forward_kinematics(self):
        _, _, joint_2 = self.get_joint_positions()
        return joint_2

    def forward_kinematics_matrix(self):
        T01 = homogeneous_transform(self.theta1, self.L1)
        T12 = homogeneous_transform(self.theta2, self.L2)
        T02 = np.dot(T01, T12)
        return (T02[0][2], T02[1][2])

    def is_reachable(self, x, y):
        distance = math.sqrt(x**2 + y **2)
        min_reach = abs(self.L1 - self.L2)
        max_reach = self.L1 + self.L2
        return distance >= min_reach and distance <= max_reach

    def convert_degree_to_radiant(self, theta):
        return np.deg2rad(theta)

