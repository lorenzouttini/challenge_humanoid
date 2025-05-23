import pybullet as p
import pybullet_data
import time
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import argparse

# Add the parent "src" directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from kinematics import inverse_kinematics, forward_kinematics
from repulsive_field import compute_repulsive_force
from plots import plot_tracking_results
from utilities import get_target_position, clamp_joint_angles, get_link_midpoints

# === Simulation constants ===

L = 1.0                                     # Link length       
CONTROL_FREQ = 50                           # Robot control loop at 50 Hz
TARGET_FREQ = 5                             # Target update frequency at 5 Hz
SIM_DURATION = 15.0                         # Simulation duration

JOINT_LIMITS = [(-np.pi, np.pi)] * 3        # Joint limits for 3 joints
obstacle_center = np.array([2*L, 0.5 * L])  # Obstacle center
# obstacle_center = np.array([L, 0.5 * L])  # Obstacle center
obstacle_radius = 0.125 * L                 # Obstacle radius



# === Main ===

def main(args):

    # Connect to PyBullet
    p.connect(p.GUI)                    # Comment for headless mode 
    # p.connect(p.DIRECT)               # Uncomment for headless mode

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Load ground and robot
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("urdf/rrr_box_robot.urdf", useFixedBase=True)
    num_joints = p.getNumJoints(robot_id)

    # Start video logging
    log_dir = os.path.join(os.path.dirname(__file__), "videos")
    os.makedirs(log_dir, exist_ok=True)
    video_path = os.path.join(log_dir, "simulation_PyBullet.mp4")
    
    video_log_id = p.startStateLogging(
        loggingType=p.STATE_LOGGING_VIDEO_MP4,
        fileName=video_path
    )

    # Camera settings
    p.resetDebugVisualizerCamera(
    cameraDistance=3.2,
    cameraYaw=60,
    cameraPitch=-45,
    cameraTargetPosition=[1.0, 0, 0]
    )

    # Set obstacle visual (red sphere)
    obstacle_visual = p.createVisualShape(
        p.GEOM_SPHERE, 
        radius=obstacle_radius, 
        rgbaColor=[1, 0, 0, 0.6]
    )
    p.createMultiBody(
        baseMass=0, 
        baseVisualShapeIndex=obstacle_visual, 
        basePosition=[*obstacle_center, 0]
    )

    # Set dt for simulation
    control_dt = 1.0 / CONTROL_FREQ             # 50 Hz
    target_dt = 1.0 / TARGET_FREQ               # 5 Hz
    sim_time = 0.0
    last_target_update = -target_dt
    
    # Initial target
    current_target = get_target_position(0)

    # Create the green target sphere (visual only)
    visual_shape_id = p.createVisualShape(
        p.GEOM_SPHERE, 
        radius=0.05, 
        rgbaColor=[0, 1, 0, 1]
    )
    target_id = p.createMultiBody(
        baseMass=0, 
        baseVisualShapeIndex=visual_shape_id, 
        basePosition=[*current_target, 0]
    )

    # Logging error for debugging
    last_valid_angles = [0.0] * num_joints
    tracking_errors = []
    per_joint_errors = []
    log_time, log_joint_actual, log_joint_target, log_ee_actual, log_ee_target = [], [], [], [], []

    # === Main Loop ===
    while sim_time < SIM_DURATION:
        # Update target position every 1/5 s
        if sim_time - last_target_update >= target_dt:
            current_target = get_target_position(sim_time)
            last_target_update = sim_time

            # Update target sphere position
            p.resetBasePositionAndOrientation(
                target_id, 
                posObj=[*current_target, 0], 
                ornObj=[0, 0, 0, 1]
            )

        # Compute repulsive force from all link midpoints
        midpoints = get_link_midpoints(last_valid_angles, L)
        total_repulse = np.zeros(2)
        eta = 0.05                              # Repulsion coefficient
        d0 = L                                  # Repulsion distance  
        for pt in midpoints:
            # Compute repulsive force from each link midpoint and accumulate
            total_repulse += compute_repulsive_force(
                pt, obstacle_center, obstacle_radius, eta=eta, d0=d0
            )

        # Compute actual end-effector position
        ee_pos = forward_kinematics(*last_valid_angles)

        # Project repulsion orthogonal to motion direction
        to_target = current_target - ee_pos
        direction = to_target / (np.linalg.norm(to_target) + 1e-6)
        parallel = np.dot(total_repulse, direction) * direction
        orthogonal = total_repulse - parallel

        # Avoid detouring towards the obstacle
        if orthogonal[0] > 0:
            orthogonal[0] *= -1

        # Define the modified target
        modified_target = current_target + orthogonal

        # Compute IK for current target
        try:
            joint_angles = inverse_kinematics(*modified_target)
            # Clamp joint angles to limits
            joint_angles = np.clip(joint_angles, -np.pi, np.pi)
            last_valid_angles = joint_angles
        except Exception as e:
            print(f"[Warning] IK failed at t={sim_time:.3f}s: {e}")
            joint_angles = last_valid_angles

        # Compute Joint States
        joint_states = p.getJointStates(robot_id, list(range(num_joints)))
        # Get current joint positions
        current_positions = np.array([s[0] for s in joint_states])
        
        # === PyBullet controller ===
        for i in range(num_joints):
            # Set joint motor control
            p.setJointMotorControl2(robot_id, i, 
                p.POSITION_CONTROL, 
                targetPosition=joint_angles[i])

        # Errors logging
        joint_errors = np.abs(joint_angles - current_positions)
        per_joint_errors.append(joint_errors.copy())

        ee_pos = forward_kinematics(*joint_angles)
        error = np.linalg.norm(ee_pos - current_target)
        tracking_errors.append(error)

        log_time.append(sim_time)
        log_joint_actual.append(current_positions.copy())
        log_joint_target.append(joint_angles.copy())
        log_ee_actual.append(ee_pos.copy())
        log_ee_target.append(current_target.copy())

        # Step the simulation
        p.stepSimulation()
        time.sleep(control_dt)
        sim_time += control_dt
    
    print("\nFinished simulation.")
    # Stop video logging
    p.stopStateLogging(video_log_id)
    p.disconnect()

    # Define the error metrics
    tracking_errors = np.array(tracking_errors)
    per_joint_errors = np.array(per_joint_errors)  
    mean_joint_errors = np.mean(per_joint_errors, axis=0)
    max_joint_errors = np.max(per_joint_errors, axis=0)
    min_joint_errors = np.min(per_joint_errors, axis=0)

    # Save the results to a .txt file
    base_dir = os.path.dirname(__file__)
    log_path = os.path.join(base_dir, "tracking_errors.txt")
    if args.write:
        with open(log_path, "a") as f:
            f.write("\n\nEnd-Effector Tracking Error for PyBullet controller:\n")
            f.write(f"Mean Error: {tracking_errors.mean()}\n")
            f.write(f"Max Error: {tracking_errors.max()}\n")
            f.write(f"Min Error: {tracking_errors.min()}\n")

            f.write("\nJoint Tracking Error Stats for PyBullet controller:\n")
            for i in range(num_joints):
                f.write(f"Joint {i+1}: Mean = {mean_joint_errors[i]:.5f}, Max = {max_joint_errors[i]:.5f}, Min = {min_joint_errors[i]:.5f}\n")


    # Plot the tracking errors
    save_path = os.path.join(os.path.dirname(__file__), "figures")
    if args.plots:
        plot_tracking_results(log_time, log_joint_actual, log_joint_target, log_ee_actual, log_ee_target, tracking_errors, save_path, PD=False)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RRR Robot Challenge Controller")
    parser.add_argument("--plots", action="store_true", help="Enable plotting")
    parser.add_argument("--write", action="store_true", help="Enable saving logs/files")
    args = parser.parse_args()

    main(args)
