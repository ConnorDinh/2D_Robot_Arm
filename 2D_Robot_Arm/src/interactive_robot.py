import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backend_bases import MouseButton
from src.robot_arm import RobotArm2D
from src.jacobian import compute_jacobian
from src.damped_ik import damped_numerical_ik

class InteractiveRobot:
    def __init__(self, robot, alpha=0.1, damping=0.1, tolerance=1e-3, max_iterations=1000, dt=0.05):
        self.robot = robot
        self.alpha = alpha
        self.damping = damping
        self.tolerance = tolerance
        self.max_iterations = max_iterations
        self.dt = dt
        self.target = None
        self.history = []
        self.frame_index = 0
        self.animation = None
        self.status = "Click to select a target"

        self.joint_velocity = np.zeros(2)
        self.end_effector_velocity = np.zeros(2)

        self.fig, (self.ax_robot, self.ax_info) = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={"width_ratios": [2, 1]})
        self.fig.subplots_adjust(wspace=0.25)
        self.setup_plot()
        self.update_display()
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

    def setup_plot(self):
        ax = self.ax_robot
        max_reach = self.robot.L1 + self.robot.L2
        min_reach = abs(self.robot.L1 - self.robot.L2)
        margin = 0.5
        ax.set_xlim(-max_reach - margin, max_reach + margin)
        ax.set_ylim(-max_reach - margin, max_reach + margin)
        ax.set_aspect("equal")
        ax.set_xlabel("X Axis")
        ax.set_ylabel("Y Axis")
        ax.set_title("Interactive 2D Robot Arm")
        ax.grid(True, alpha=0.3)

        outer = plt.Circle((0, 0), max_reach, fill=False, linestyle="--", color="#16A085", alpha=0.5)
        ax.add_patch(outer)

        if min_reach > 0:
            inner = plt.Circle((0, 0), min_reach, fill=False, linestyle="--", color="#D9361E", alpha=0.5)
            ax.add_patch(inner)

        self.robot_line, = ax.plot([], [], "-o",linewidth=4,color="#4D8DBA",markersize=10,markerfacecolor="#FFEFC4",label="Robot Arm")

        self.target_marker, = ax.plot([], [], "x",color="#D9361E",markersize=12,markeredgewidth=2,label="Target")

        self.trajectory_line, = ax.plot([], [], "--",color="#16A085",linewidth=2,label="End-Effector Path")

        self.velocity_arrow = ax.quiver([], [], [], [], angles="xy", scale_units="xy", scale=1, color="#D9361E")

        ax.legend(loc="upper right")

        self.ax_info.axis("off")
        self.info_text = self.ax_info.text(0.03, 0.98, "", transform=self.ax_info.transAxes,va="top",ha="left",fontsize=10,family="monospace",linespacing=1.5)

    def update_display(self):
        base, joint1, joint2 = self.robot.get_joint_positions()

        x = [base[0], joint1[0], joint2[0]]
        y = [base[1], joint1[1], joint2[1]]

        self.robot_line.set_data(x, y)

        J = compute_jacobian(self.robot)

        if self.target is None:
            target_text = "Not selected"
            error_text = "N/A"
        else:
            target_text = (f"({self.target[0]:.3f}, "f"{self.target[1]:.3f})")
            error = np.linalg.norm(np.array(self.target) - np.array(joint2))
            error_text = f"{error:.5f}"

        speed = np.linalg.norm(self.end_effector_velocity)

        info = (
            "ROBOT TELEMETRY\n"
            "----------------------------\n"
            f"Status: {self.status}\n\n"
            f"Target:\n  {target_text}\n\n"
            f"End Effector:\n"
            f"  ({joint2[0]:.3f}, {joint2[1]:.3f})\n\n"
            f"Joint Angles:\n"
            f"  Theta 1: {np.degrees(self.robot.theta1):.2f} deg\n"
            f"  Theta 2: {np.degrees(self.robot.theta2):.2f} deg\n\n"
            f"Joint Velocities:\n"
            f"  Theta 1: {self.joint_velocity[0]:.3f} rad/s\n"
            f"  Theta 2: {self.joint_velocity[1]:.3f} rad/s\n\n"
            f"End-Effector Velocity:\n"
            f"  Vx: {self.end_effector_velocity[0]:.3f}\n"
            f"  Vy: {self.end_effector_velocity[1]:.3f}\n"
            f"  Speed: {speed:.3f}\n\n"
            f"Jacobian:\n"
            f"  [{J[0, 0]:7.3f} {J[0, 1]:7.3f}]\n"
            f"  [{J[1, 0]:7.3f} {J[1, 1]:7.3f}]\n\n"
            f"Det(J): {np.linalg.det(J):.5f}\n"
            f"Position Error: {error_text}"
        )

        self.info_text.set_text(info)

        self.velocity_arrow.set_offsets(np.array([[joint2[0], joint2[1]]]))
        self.velocity_arrow.set_UVC(self.end_effector_velocity[0], self.end_effector_velocity[1])

        self.fig.canvas.draw_idle()

    def stop_animation(self):
        if self.animation is not None:
            if self.animation.event_source is not None:
                self.animation.event_source.stop()
            self.animation = None

    def on_click(self, event):
        if event.inaxes != self.ax_robot:
            return

        if event.button != MouseButton.LEFT:
            return

        if event.xdata is None or event.ydata is None:
            return

        target_x = float(event.xdata)
        target_y = float(event.ydata)

        if not self.robot.is_reachable(target_x, target_y):
            print(f"Target ({target_x:.3f}, {target_y:.3f}) " "is unreachable!")
            self.status = "Unreachable target"
            self.update_display()
            return

        self.stop_animation()

        self.target = (target_x, target_y)
        self.target_marker.set_data([target_x], [target_y])

        initial_angles = (self.robot.theta1, self.robot.theta2)

        history = []

        success = damped_numerical_ik(
            self.robot,
            target_x,
            target_y,
            alpha=self.alpha,
            damping=self.damping,
            tolerance=self.tolerance,
            max_iterations=self.max_iterations,
            history=history
        )

        self.robot.set_joint_angles(*initial_angles)

        if not history:
            history = [initial_angles]

        self.history = history
        self.frame_index = 0
        self.solver_success = success

        self.joint_velocity = np.zeros(2)
        self.end_effector_velocity = np.zeros(2)

        self.trajectory_line.set_data([], [])
        self.status = "Moving" if len(history) > 1 else (
            "Reached" if success else "Solver failed"
        )

        self.update_display()

        if len(history) <= 1:
            return

        self.animation = FuncAnimation(
            self.fig,
            self.update_frame,
            frames=len(history),
            interval=self.dt * 1000,
            repeat=False,
            blit=False
        )

        self.fig.canvas.draw_idle()

    def update_frame(self, frame):
        self.frame_index = frame

        theta1, theta2 = self.history[frame]

        if frame == 0:
            self.joint_velocity = np.zeros(2)
        else:
            previous = np.array(self.history[frame - 1])
            current = np.array([theta1, theta2])

            self.joint_velocity = (
                current - previous
            ) / self.dt

        self.robot.set_joint_angles(theta1, theta2)

        J = compute_jacobian(self.robot)

        self.end_effector_velocity = (
            J @ self.joint_velocity
        )

        # Update trajectory
        trajectory_x = []
        trajectory_y = []

        for t1, t2 in self.history[:frame + 1]:
            x = (
                self.robot.L1 * np.cos(t1)
                + self.robot.L2 * np.cos(t1 + t2)
            )
            y = (
                self.robot.L1 * np.sin(t1)
                + self.robot.L2 * np.sin(t1 + t2)
            )

            trajectory_x.append(x)
            trajectory_y.append(y)

        self.trajectory_line.set_data(
            trajectory_x,
            trajectory_y
        )

        if frame == len(self.history) - 1:
            self.status = (
                "Reached"
                if self.solver_success
                else "Solver failed"
            )
            self.joint_velocity = np.zeros(2)
            self.end_effector_velocity = np.zeros(2)

        self.update_display()

        return (
            self.robot_line,
            self.target_marker,
            self.trajectory_line,
            self.velocity_arrow,
            self.info_text
        )

    def show(self):
        plt.show()


def main():
    robot = RobotArm2D(2, 1)

    robot.set_joint_angles(
        np.deg2rad(30),
        np.deg2rad(45)
    )

    app = InteractiveRobot(
        robot,
        alpha=0.1,
        damping=0.1,
        tolerance=1e-3,
        max_iterations=1000,
        dt=0.05
    )

    app.show()


if __name__ == "__main__":
    main()
