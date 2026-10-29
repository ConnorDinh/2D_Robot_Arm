from src.robot_arm import RobotArm2D
import numpy as np
from src.inverse_kinematics import analytical_ik

TEST_CASES = [(3, 0), (2, 1), (0, 3), (1, 0), (0, 0), (4, 0)]
REACHABLE = [True, True, True, True, False, False]
def test_analytical_ik():
    robot = RobotArm2D(2, 1)
    for i in range(len(TEST_CASES)):
        target_position = TEST_CASES[i]
        solutions = analytical_ik(robot, target_position[0], target_position[1])
        if not REACHABLE[i]:
            assert solutions == None
        else:
            for solution in solutions:
                robot.set_joint_angles(solution[0], solution[1])
                actual_position = robot.forward_kinematics()
                np.testing.assert_allclose(actual_position, target_position, rtol=1e-6, atol=1e-6)
