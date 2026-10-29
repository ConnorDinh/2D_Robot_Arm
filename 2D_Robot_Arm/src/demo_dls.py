from src.robot_arm import RobotArm2D
from src.numerical_ik import numerical_ik
from src.damped_ik import damped_numerical_ik
from src.plot_joint_update_comparison import plot_joint_update_comparison

# Create two identical robots
robot_pinv = RobotArm2D(2, 1)
robot_dls = RobotArm2D(2, 1)

# Start near a singularity
theta1 = 0
theta2 = robot_pinv.convert_degree_to_radiant(1)

robot_pinv.set_joint_angles(theta1, theta2)
robot_dls.set_joint_angles(theta1, theta2)

# Target
target = (2.5, 0.5)

# Record joint configurations
pinv_history = []
dls_history = []

# Run pseudoinverse IK
pinv_success = numerical_ik(
    robot_pinv,
    target[0],
    target[1],
    alpha=0.1,
    tolerance=1e-3,
    max_iterations=1000,
    history=pinv_history
)

# Run DLS IK
dls_success = damped_numerical_ik(
    robot_dls,
    target[0],
    target[1],
    alpha=0.1,
    damping=0.5,
    tolerance=1e-3,
    max_iterations=1000,
    history=dls_history
)

print("Pseudoinverse converged:", pinv_success)
print("DLS converged:", dls_success)

print("Pseudoinverse iterations:", len(pinv_history) - 1)
print("DLS iterations:", len(dls_history) - 1)

# Compare joint updates
plot_joint_update_comparison(pinv_history, dls_history)