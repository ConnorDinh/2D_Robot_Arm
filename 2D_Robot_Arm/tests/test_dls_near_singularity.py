from src.damped_ik import compute_dls_update
from src.robot_arm import RobotArm2D
from src.jacobian import compute_jacobian
import numpy as np

def test_dls_near_singularity():
    robot = RobotArm2D(2, 1)
    robot.set_joint_angles(0, robot.convert_degree_to_radiant(1))
    error = [0.1, 0]
    J = compute_jacobian(robot)
    pinv_update = 0.1 * np.linalg.pinv(J) @ np.array(error)
    dls_update = compute_dls_update(robot, error, 0.1, 0.1)
    assert np.linalg.norm(dls_update) < np.linalg.norm(pinv_update)