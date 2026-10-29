from src.robot_arm import RobotArm2D
from src.numerical_ik import numerical_ik
from src.animate_numerical_ik import animate_numerical_ik

robot = RobotArm2D(2, 1)

robot.set_joint_angles(
    robot.convert_degree_to_radiant(30),
    robot.convert_degree_to_radiant(45)
)

target = (3.0, 0.0)
history = []

success = numerical_ik(
    robot,
    target[0],
    target[1],
    alpha=0.1,
    tolerance=1e-3,
    max_iterations=1000,
    history=history
)

print("Success:", success)
print("Iterations:", len(history) - 1)
print("Final position:", robot.forward_kinematics())

if history:
    animate_numerical_ik(robot, history, target)