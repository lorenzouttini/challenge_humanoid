import numpy as np


# Constants
JOINT_LIMITS = [(-np.pi, np.pi)] * 3        # Joint limits for 3 joints
L = 1.0                                     # Link length 


def get_target_position(t, L=L, f=5.0):
    """
    Generate a target position in a sinusoidal trajectory.
    """
    x = 2 * L
    y = L * np.sin(2 * np.pi * f * t)
    return np.array([x, y])



def clamp_joint_angles(joint_angles):
    """
    Clamp joint angles to their respective limits.
    """
    return np.clip(joint_angles,
                   [limit[0] for limit in JOINT_LIMITS],
                   [limit[1] for limit in JOINT_LIMITS])



def get_link_midpoints(joint_angles, link_length=L):
    """
    Calculate the midpoints of the links based on joint angles.
    """
    θ1, θ2, θ3 = joint_angles
    p0 = np.array([0, 0])
    p1 = p0 + link_length * np.array([np.cos(θ1), np.sin(θ1)])
    p2 = p1 + link_length * np.array([np.cos(θ1 + θ2), np.sin(θ1 + θ2)])
    p3 = p2 + link_length * np.array([np.cos(θ1 + θ2 + θ3), np.sin(θ1 + θ2 + θ3)])
    
    mid1 = (p0 + p1) / 2
    mid2 = (p1 + p2) / 2
    mid3 = (p2 + p3) / 2
    return [mid1, mid2, mid3]