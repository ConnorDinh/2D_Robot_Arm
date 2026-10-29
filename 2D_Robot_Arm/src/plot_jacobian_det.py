import numpy as np
import matplotlib.pyplot as plt
def plot_jacobian_det(robot):
    theta2s = list(range(-180, 180, 5))
    res = []
    for theta2 in theta2s:
        det = robot.L1 * robot.L2 * np.sin(robot.convert_degree_to_radiant(theta2))
        res.append((theta2, det))
    fig, ax = plt.subplots(figsize=(5, 5))
    
    for i in range(len(res)):
        ax.plot(res[i][0], res[i][1], '-o', color="#16A085", markersize=10, markerfacecolor="#FFEFC4")
    ax.axhline(y=0, color='r', linestyle='--', linewidth=2)
    ax.set_xlim(xmin=-180, xmax=180)
    ax.set_ylim(ymin=-2.5, ymax=2.5)
    ax.set_title("Theta 2 vs Determinant")
    ax.set_xlabel("Theta 2")
    ax.set_ylabel("Determinant")
    plt.show()
