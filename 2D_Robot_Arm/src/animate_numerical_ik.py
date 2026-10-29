import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def animate_numerical_ik(robot, history, target):
    fig, ax = plt.subplots(figsize=(6,6))
    ax.plot(target[0], target[1], 'x', linewidth=4, color="#D9361E", markersize=12, label="Target")

    robot_line, = ax.plot(
        [], [], '-o',
        linewidth=4,
        color="#4D8DBA",
        markersize=10,
        markerfacecolor="#FFEFC4",
        label="Robot Arm"
    )
    info_text = ax.text(
        0.02, 0.98,
        "",
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top",
        family="monospace",
        bbox=dict(
            facecolor="white",
            alpha=0.85,
            edgecolor="gray"
        )
    )
    trajectory_line, = ax.plot(
        [], [],
        '--',
        color="#16A085",
        linewidth=2,
        label="End-Effector Path"
    )
    def update(frame):
        theta1, theta2 = history[frame]
        robot.set_joint_angles(theta1, theta2)
        base, joint1, joint2 = robot.get_joint_positions()
        x_coor = [base[0], joint1[0], joint2[0]]
        y_coor = [base[1], joint1[1], joint2[1]]
        robot_line.set_data(x_coor, y_coor)
        end_x, end_y = joint2

        error = np.linalg.norm(
            np.array(target) - np.array([end_x, end_y])
        )

        info_text.set_text(
            f"Target: ({target[0]:.3f}, {target[1]:.3f})\n"
            f"End Effector: ({end_x:.3f}, {end_y:.3f})\n"
            f"Theta 1: {np.degrees(theta1):.2f} deg\n"
            f"Theta 2: {np.degrees(theta2):.2f} deg\n"
            f"Error: {error:.5f}"
        )
        trajectory_x = []
        trajectory_y = []

        for t1, t2 in history[:frame + 1]:
            x = robot.L1 * np.cos(t1) + robot.L2 * np.cos(t1 + t2)
            y = robot.L1 * np.sin(t1) + robot.L2 * np.sin(t1 + t2)

            trajectory_x.append(x)
            trajectory_y.append(y)

        trajectory_line.set_data(trajectory_x, trajectory_y)
        return robot_line, info_text, trajectory_line

    ax.set_aspect("equal")

    maximum_reach = robot.L1 + robot.L2
    margin = 0.5

    ax.set_xlim(-maximum_reach - margin, maximum_reach + margin)
    ax.set_ylim(-maximum_reach - margin, maximum_reach + margin)

    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.grid(True)
    ax.legend()

    # Create animation
    ani = FuncAnimation(
        fig,
        update,
        frames=len(history),
        interval=50,
        repeat=False
    )

    plt.show()
    return ani