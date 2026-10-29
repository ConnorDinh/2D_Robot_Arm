from src.robot_arm import RobotArm2D
from src.singularity import is_singular

TEST_CASES = [(0, 0), (30, 0), (0, 90), (45, 180), (-30, -90), (60, 45)]
EXPECTED_RESULTS = [True, True, False, True, False, False]

def test_singularity():
    robot = RobotArm2D(2, 1)
    for i in range(len(TEST_CASES)):
        theta1 = robot.convert_degree_to_radiant(TEST_CASES[i][0])
        theta2 = robot.convert_degree_to_radiant(TEST_CASES[i][1])
        robot.set_joint_angles(theta1, theta2)
        actual = is_singular(robot, 1e-6)
        expected = EXPECTED_RESULTS[i]
        assert actual == expected

def test_near_singularity():
    robot = RobotArm2D(2, 1)
    theta1 = 0
    theta2 = 0.000001
    robot.set_joint_angles(theta1, theta2)
    assert is_singular(robot, 1e-5) == True
    assert is_singular(robot, 1e-7) == False

def test_singularity_independent_of_theta1():
    robot = RobotArm2D(2, 1)
    for theta1 in range(-180, 180, 5):
        robot.set_joint_angles(theta1, 0)
        assert is_singular(robot, 1e-6) ==True

test_singularity()
test_near_singularity()
test_singularity_independent_of_theta1()