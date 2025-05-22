import numpy as np

def compute_repulsive_force(end_effector_pos, obstacle_center, obstacle_radius, eta=1.0, d0=0.25):
    """
    Compute the repulsive force applied to the end-effector to avoid the obstacle.

    Parameters:
    - end_effector_pos: np.array([x, y])
    - obstacle_center: np.array([x0, y0])
    - obstacle_radius: scalar 
    - eta: strength of repulsion
    - d0: influence distance (only compute repulsion within this range)

    Returns:
    - repulsive_force: np.array([Fx, Fy])
    """
    diff = end_effector_pos - obstacle_center
    dist = np.linalg.norm(diff)
    effective_dist = dist - obstacle_radius

    if effective_dist > d0:
        return np.zeros(2)  # No force applied

    if effective_dist <= 0:
        # Inside obstacle — force hard push outward
        return eta * diff / (dist + 1e-6) * 100.0

    force_mag = eta * (1.0 / effective_dist - 1.0 / d0) / (effective_dist ** 2)
    direction = diff / dist
    return force_mag * direction
