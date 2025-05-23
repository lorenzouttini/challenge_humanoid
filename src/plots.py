import matplotlib.pyplot as plt
import numpy as np
import os

def plot_tracking_results(log_time, log_joint_actual, log_joint_target, 
                          log_ee_actual, log_ee_target, tracking_errors, 
                          save_path=None, PD=False):
    """
    Plot the tracking results for joint angles and end-effector position.
    """

    log_time = np.array(log_time)
    log_joint_actual = np.array(log_joint_actual)
    log_joint_target = np.array(log_joint_target)
    log_ee_actual = np.array(log_ee_actual)
    log_ee_target = np.array(log_ee_target)


    # === Joint tracking plots ===
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    joint_labels = ['Joint 1', 'Joint 2', 'Joint 3']

    # Plot joint angles
    for i in range(3):
        axs[i].plot(log_time, log_joint_target[:, i], color='orange', linestyle='--', marker='o', label='Target')
        axs[i].plot(log_time, log_joint_actual[:, i], color='blue', linestyle='-', label='Actual')
        axs[i].set_ylabel(f"{joint_labels[i]} (rad)")
        axs[i].legend()
        axs[i].grid(True)

    axs[2].set_xlabel("Time (s)")
    plt.tight_layout()
    if save_path:
        if PD:
            # plt.title("Joint Angle Tracking Over Time - PD Controller")
            fig.savefig(os.path.join(save_path, "joint_tracking_PD.png"))
        else:
            # plt.title("Joint Angle Tracking Over Time - PyBullet Controller")
            fig.savefig(os.path.join(save_path, "joint_tracking_PyBullet.png"))


    # === End-effector tracking error plot ===
    fig2 = plt.figure(figsize=(8, 4))
    plt.plot(tracking_errors, color='blue')
    plt.xlabel("Time Step")
    plt.ylabel("Error (Euclidean distance)")
    plt.grid(True)

    title = "End-Effector Tracking Error Over Time"
    plt.title(title)
    plt.tight_layout()

    if save_path:
        filename = "ee_tracking_error_PD.png" if PD else "ee_tracking_error_PyBullet.png"
        plt.savefig(os.path.join(save_path, filename))
