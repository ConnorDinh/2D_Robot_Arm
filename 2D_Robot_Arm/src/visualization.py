import matplotlib.pyplot as plt
from src.inverse_kinematics import analytical_ik
def draw_robot(robot):
    base, joint1, joint2 = robot.get_joint_positions()
    origin_theta_1, origin_theta_2 = robot.theta1, robot.theta2 
    fig, ax = plt.subplots(figsize=(5, 5))

    x_coordinates = [base[0], joint1[0], joint2[0]]
    y_coordinates = [base[1], joint1[1], joint2[1]]

    ax.plot(x_coordinates, y_coordinates, '-o', linewidth=4, color="#4D8DBA", markersize=10, markerfacecolor="#FFEFC4")
    ax.set_aspect('equal')

    maximum_reach = robot.L1 + robot.L2
    margin = 0.5
    x_min = -maximum_reach - margin
    x_max = maximum_reach + margin
    y_min = -maximum_reach - margin
    y_max = maximum_reach + margin

    x_pos = []
    y_pos = []
    for theta1 in range(-180, 180, 5):
        for theta2 in range(-180, 180, 5):
            angle_1 = robot.convert_degree_to_radiant(theta1)
            angle_2 = robot.convert_degree_to_radiant(theta2)
            robot.set_joint_angles(angle_1, angle_2)
            x_pos.append(robot.forward_kinematics()[0])
            y_pos.append(robot.forward_kinematics()[1])

    robot.set_joint_angles(origin_theta_1, origin_theta_2)
    ax.set_xlim(xmin=x_min, xmax=x_max)
    ax.set_ylim(ymin=y_min, ymax=y_max)

    ax.set_title("Standard 2D Robot Arm")
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.grid(True)

    ax.scatter(x_pos, y_pos, color="#97D7FF")
    plt.show()

def draw_ik_solutions(robot, target):
    origin_theta_1, origin_theta_2 = robot.theta1, robot.theta2 
    colors =["#4D8DBA", "#16A085"]
    solutions = analytical_ik(robot, target[0], target[1])
    fig, ax = plt.subplots(figsize=(5, 5))

    if solutions is None:
        plt.text(0.25, 0.5, "Target is unreachable!", fontsize=12, color='#D9361E', weight='bold')
        return
    
    for i in range(len(solutions)):
        robot.set_joint_angles(solutions[i][0], solutions[i][1])
        base, joint1, joint2 = robot.get_joint_positions()
        x_coordinates = [base[0], joint1[0], joint2[0]]
        y_coordinates = [base[1], joint1[1], joint2[1]]
        ax.plot(x_coordinates, y_coordinates, '-o', linewidth=4, color=colors[i], markersize=10, markerfacecolor="#FFEFC4", label=f"Solution {i}")

    ax.plot(target[0], target[1], 'x', color="#D9361E", markersize=12, label="Target")
    ax.set_aspect('equal')
    maximum_reach = robot.L1 + robot.L2
    margin = 0.5
    x_min = -maximum_reach - margin
    x_max = maximum_reach + margin
    y_min = -maximum_reach - margin
    y_max = maximum_reach + margin
    robot.set_joint_angles(origin_theta_1, origin_theta_2)
    ax.set_xlim(xmin=x_min, xmax=x_max)
    ax.set_ylim(ymin=y_min, ymax=y_max)
    ax.set_title("Standard 2D Robot Arm Inverse Kinematics")
    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.legend()
    plt.show()