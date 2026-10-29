from src.numerical_ik import numerical_ik
from src.robot_arm import RobotArm2D
import numpy as np

TARGET = [[2,1], [1.5,2], [0,2.5], [3,0], [4,0], [0,0]]
ACTUAL = [True, True, True, True, False, False]
def test_numerical_ik():
    for i in range(len(TARGET)):
        robot = RobotArm2D(2, 1)
        robot.set_joint_angles(robot.convert_degree_to_radiant(30), robot.convert_degree_to_radiant(45))
        expected = numerical_ik(robot, TARGET[i][0], TARGET[i][1], 0.1, 1e-3, 1000)
        assert expected == ACTUAL[i]
        if robot.is_reachable(TARGET[i][0], TARGET[i][1]) and expected:
            final_position = np.array(list(robot.forward_kinematics()))
            assert np.linalg.norm(final_position - np.array(TARGET[i])) < 1e-3
        