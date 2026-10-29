import numpy as np
import matplotlib.pyplot as plt


def plot_joint_update_comparison(pinv_history, dls_history):

    # Convert joint-angle histories into NumPy arrays
    pinv_history = np.asarray(pinv_history)
    dls_history = np.asarray(dls_history)

    # Calculate differences between consecutive joint configurations
    pinv_delta = np.diff(pinv_history, axis=0)
    dls_delta = np.diff(dls_history, axis=0)

    # Calculate magnitude of each joint update
    pinv_magnitude = np.linalg.norm(pinv_delta, axis=1)
    dls_magnitude = np.linalg.norm(dls_delta, axis=1)

    # Create plot
    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(
        range(1, len(pinv_magnitude) + 1),
        pinv_magnitude,
        color="#4D8DBA",
        linewidth=2,
        label="Jacobian Pseudoinverse"
    )

    ax.plot(
        range(1, len(dls_magnitude) + 1),
        dls_magnitude,
        color="#16A085",
        linewidth=2,
        label="Damped Least Squares"
    )

    ax.set_title("Joint Update Magnitude Comparison")
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Joint Update Magnitude (rad)")

    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.show()