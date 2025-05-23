import numpy as np

L = 1.0  # link length

def forward_kinematics(theta1, theta2, theta3):
    """
    Compute (x, y) of the end-effector given joint angles in radians.
    """
    # Compute positions of first joint
    x1 = L * np.cos(theta1)
    y1 = L * np.sin(theta1)

    # Compute positions of second joint
    x2 = L * np.cos(theta1 + theta2)
    y2 = L * np.sin(theta1 + theta2)

    # Compute positions of third joint
    x3 = L * np.cos(theta1 + theta2 + theta3)
    y3 = L * np.sin(theta1 + theta2 + theta3)

    # Compute end-effector position
    x = x1 + x2 + x3
    y = y1 + y2 + y3

    return np.array([x, y])

def inverse_kinematics(x_target, y_target, phi=0.0):
    """
    Solves IK for a 3-link planar arm to reach (x, y) with end-effector orientation phi.
    """
    # Compute wrist position (end of link 2)
    x_wrist = x_target - L * np.cos(phi)
    y_wrist = y_target - L * np.sin(phi)

    # Distance from base to wrist
    r2 = x_wrist**2 + y_wrist**2
    r = np.sqrt(r2)

    # Law of cosines for angle at joint2
    cos_theta2 = (r2 - L**2 - L**2) / (2 * L * L)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = np.arccos(cos_theta2)

    # Solve for theta1
    k1 = L + L * cos_theta2
    k2 = L * np.sin(theta2)
    theta1 = np.arctan2(y_wrist, x_wrist) - np.arctan2(k2, k1)

    # Solve for theta3 (end-effector orientation)
    theta3 = phi - (theta1 + theta2)

    return np.array([theta1, theta2, theta3])

