from src.robot_arm import RobotArm2D
import numpy as np

TEST_CASES = [(0, 0), (90, 0), (0, 90), (0, 180), (-90, 0)]
def test_matrix_forward_kinematics():
    robot = RobotArm2D(2, 1)
    for angles in TEST_CASES:
        angle_0 = robot.convert_degree_to_radiant(angles[0])
        angle_1 = robot.convert_degree_to_radiant(angles[1])
        robot.set_joint_angles(angle_0, angle_1)
        positrion_trig = robot.forward_kinematics()
        positrion_matrix = robot.forward_kinematics_matrix()
        np.testing.assert_allclose(positrion_trig, positrion_matrix, rtol=1e-6, atol=1e-6)

test_matrix_forward_kinematics()
