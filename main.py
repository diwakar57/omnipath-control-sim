"""
OmniPath-DSP: Autonomous Trajectory Tracking Simulation
Workflow: 
1. Path Definition -> 2. State Sensing -> 3. PID Calculation -> 4. Physics Update
"""

import numpy as np
import matplotlib.pyplot as plt

# Internal Project Imports
from models.bicycle_model import BicycleRobot
from controllers.pid_controller import PIDController
from utils.path_generator import Path

def run_control_simulation():
    # --- SECTION 1: HYPERPARAMETERS & CONFIG ---
    dt = 0.05            # Sampling time (20Hz)
    total_time = 20.0    # Total simulation seconds
    steps = int(total_time / dt)
    
    # --- SECTION 2: INITIALIZATION ---
    # Start robot at y=2.0 (2 meters above the target path)
    robot = BicycleRobot(x=0.0, y=2.0, theta=0.0, v=2.0, wheelbase=2.5)
    
    # Target Path: A straight line sitting exactly on the X-axis (y=0)
    target_path = Path(start_point=[0, 0], end_point=[100, 0])
    
    # PID Gains: 
    # Kp: Proportional (Initial turn)
    # Ki: Integral (Corrects steady-state offset)
    # Kd: Derivative (Dampens the "wobble")
    pid = PIDController(kp=0.6, ki=0.01, kd=0.3)

    # Data logging for plotting
    x_hist, y_hist = [], []

    # --- SECTION 3: THE CONTROL LOOP (The "Brain") ---
    plt.ion() # Interactive mode for real-time animation
    fig, ax = plt.subplots(figsize=(10, 5))

    for _ in range(steps):
        # A. PERCEPTION: Calculate Cross-Track Error (CTE)
        # How far is the robot (y) from the target path (y=0)?
        cte = target_path.get_cte([robot.x, robot.y])
        
        # B. PLANNING: PID Logic
        # The controller output is our desired steering angle (delta)
        steering_cmd = pid.control(cte, dt)
        
        # C. ACTUATION LIMITS: Prevent "impossible" steering angles
        # Real cars can't turn wheels 90 degrees. Limit to ±30°
        max_steer = np.deg2rad(30.0)
        steering_cmd = np.clip(steering_cmd, -max_steer, max_steer)
        
        # D. PHYSICS: Update the robot state
        robot.update(steering_angle=steering_cmd, acceleration=0.0, dt=dt)

        # Log Data
        x_hist.append(robot.x)
        y_hist.append(robot.y)

        # --- SECTION 4: REAL-TIME VISUALIZATION ---
        ax.clear()
        # Draw the target goal (Red Dashed Line)
        ax.axhline(y=0, color='r', linestyle='--', label="Target Path (y=0)")
        # Draw the actual driven path (Blue Line)
        ax.plot(x_hist, y_hist, "b-", linewidth=2, label="Robot Trajectory")
        # Draw the current robot position (Red Dot)
        ax.plot(robot.x, robot.y, "ro", markersize=8)
        
        ax.set_ylim(-3, 3) # Keep focus on the recovery zone
        ax.set_xlim(0, max(10, robot.x + 5))
        ax.set_xlabel("X Position [m]")
        ax.set_ylabel("Y Position [m]")
        ax.set_title(f"OmniPath-DSP: PID Recovery (CTE: {cte:.2f}m)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)
        
        plt.pause(0.01)

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    run_control_simulation()