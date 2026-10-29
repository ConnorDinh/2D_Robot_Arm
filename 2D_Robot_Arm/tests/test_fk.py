from src.robot_arm import RobotArm2D
import numpy as np

TEST_CASES = [(0, 0), (90, 0), (0, 90), (0, 180), (-90, 0)]
EXPECTED_RESULTS = [(3, 0), (0, 3), (2, 1), (1, 0), (0, -3)]
def test_forward_kinematics():
    robot = RobotArm2D(2, 1)
    for i in range(len(TEST_CASES)):
        rad_theta_1 = np.deg2rad(TEST_CASES[i][0])
        rad_theta_2 = np.deg2rad(TEST_CASES[i][1])
        robot.set_joint_angles(rad_theta_1, rad_theta_2)
        actual = robot.forward_kinematics()
        expect = EXPECTED_RESULTS[i]
        np.testing.assert_allclose(actual, expect, rtol=1e-6, atol=1e-6)

test_forward_kinematics()
