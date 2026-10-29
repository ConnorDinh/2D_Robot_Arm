from src.jacobian import end_effector_velocity
import matplotlib.pyplot as plt
import numpy as np

def plot_end_effector_velocity(robot, theta1_dot, theta2_dot):
    base, joint1, joint2 = robot.get_joint_positions()
    velocity = end_effector_velocity(robot, theta1_dot, theta2_dot)
    fig, ax = plt.subplots(figsize=(5, 5))
    x_coor = [base[0], joint1[0], joint2[0]]
    y_coor = [base[1], joint1[1], joint2[1]]
    ax.plot(x_coor, y_coor, '-o', linewidth=4, color="#4D8DBA", markersize=10, markerfacecolor="#FFEFC4")

    ax.quiver(joint2[0], joint2[1], velocity[0], velocity[1], angles='xy', scale_units='xy', scale=1, color="#D9361E")
    ax.set_aspect('equal')
    maximum_reach = robot.L1 + robot.L2
    margin = 0.5
    x_min = -maximum_reach - margin
    x_max = maximum_reach + margin
    y_min = -maximum_reach - margin
    y_max = maximum_reach + margin
    ax.set_xlim(xmin=x_min, xmax=x_max)
    ax.set_ylim(ymin=y_min, ymax=y_max)
    ax.set_title(f"End Effector Velocity | Speed: {np.linalg.norm(velocity):.2f}")
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.grid(True)
    plt.show()