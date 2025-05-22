import pybullet as p
import pybullet_data
import time
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import argparse


sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from kinematics import inverse_kinematics, forward_kinematics
from plots import plot_tracking_results
from utilities import get_target_position

# === Simulation constants ===

L = 1.0                    # Link length
TARGET_FREQ = 30.0         # Target updates at 30 Hz
CONTROL_FREQ = 1000.0      # Robot control loop at 1 kHz
SIM_DURATION = 10.0        # seconds


# === Main ===

def main(args):

    # Connect to PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    # Load ground and robot
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(os.path.join("..", "urdf", "rrr_box_robot.urdf"), basePosition=[0, 0, 0], useFixedBase=True)
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

    # Initialize timing
    control_dt = 1.0 / CONTROL_FREQ
    target_dt = 1.0 / TARGET_FREQ
    sim_time = 0.0
    last_target_update = -target_dt

    # Initial target
    current_target = get_target_position(0)

    # Create the green target sphere (visual only)
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=[0, 1, 0, 1],  # Green
    )

    target_id = p.createMultiBody(
        baseMass=0,  # Static
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[current_target[0], current_target[1], 0],
    )

    # Logging error for debugging
    tracking_errors = []
    per_joint_errors = []
    log_time, log_joint_actual, log_joint_target, log_ee_actual, log_ee_target = [], [], [], [], []


    # === Main Loop ===
    while sim_time < SIM_DURATION:
        # Update target position every 1/30 s
        if sim_time - last_target_update >= target_dt:
            current_target = get_target_position(sim_time)
            last_target_update = sim_time

            # Update target sphere position (in XY plane, Z = 0)
            p.resetBasePositionAndOrientation(
                target_id,
                posObj=[current_target[0], current_target[1], 0],
                ornObj=[0, 0, 0, 1]
            )

        # Compute IK for current target
        try:
            joint_angles = inverse_kinematics(*current_target)
        except ValueError as e:
            print(f"[Warning] IK failed at t={sim_time:.3f}s: {e}")
            joint_angles = [0.0] * num_joints  # fallback
        
        # Compute actual end-effector position
        ee_pos = forward_kinematics(*joint_angles)
        joint_states = p.getJointStates(robot_id, list(range(num_joints)))
        current_positions = np.array([s[0] for s in joint_states])

        # === PyBullet controller ===
        for i in range(num_joints):
            p.setJointMotorControl2(robot_id, i, 
                p.POSITION_CONTROL, 
                targetPosition=joint_angles[i])

        # Errors logging
        joint_errors = np.abs(joint_angles - current_positions)
        per_joint_errors.append(joint_errors.copy())

        error = np.linalg.norm(ee_pos - current_target)
        tracking_errors.append(error)

        log_time.append(sim_time)
        log_joint_actual.append(current_positions.copy())
        log_joint_target.append(joint_angles.copy())
        log_ee_actual.append(ee_pos.copy())
        log_ee_target.append(current_target.copy())

        # Step simulation
        p.stepSimulation()
        time.sleep(control_dt)
        sim_time += control_dt

    print("Finished simulation.")
    # Stop video logging
    p.stopStateLogging(video_log_id)
    p.disconnect()

    # Summary
    tracking_errors = np.array(tracking_errors)
    per_joint_errors = np.array(per_joint_errors)  # shape: [timesteps, 3]
    mean_joint_errors = np.mean(per_joint_errors, axis=0)
    max_joint_errors = np.max(per_joint_errors, axis=0)
    min_joint_errors = np.min(per_joint_errors, axis=0)


    base_dir = os.path.dirname(__file__)
    log_path = os.path.join(base_dir, "tracking_errors.txt")
    # if write:
    if args.write:
        with open(log_path, "a") as f:
            f.write("\n\nEnd-Effector Tracking Error for PyBullet controller:\n")
            f.write(f"Mean Error: {tracking_errors.mean()}\n")
            f.write(f"Max Error: {tracking_errors.max()}\n")
            f.write(f"Min Error: {tracking_errors.min()}\n")

            f.write("\nJoint Tracking Error Stats for PyBullet controller:\n")
            for i in range(num_joints):
                f.write(f"Joint {i+1}: Mean = {mean_joint_errors[i]:.5f}, Max = {max_joint_errors[i]:.5f}, Min = {min_joint_errors[i]:.5f}\n")


    # Plots
    save_path = os.path.join(os.path.dirname(__file__), "figures")
    # if plots:
    if args.plots:
        plot_tracking_results(log_time, log_joint_actual, log_joint_target, log_ee_actual, log_ee_target, tracking_errors, save_path, PD=False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RRR Robot Challenge Controller")
    parser.add_argument("--plots", action="store_true", help="Enable plotting")
    parser.add_argument("--write", action="store_true", help="Enable saving logs/files")
    args = parser.parse_args()

    main(args)
