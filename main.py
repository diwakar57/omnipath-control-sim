import numpy as np
import matplotlib.pyplot as plt

# Internal Project Imports
from models.bicycle_model import BicycleRobot
from controllers.pid_controller import PIDController
from utils.path_generator import Path
from sensors.camera_sensor import CameraSensor
from sensors.reliability_monitor import ReliabilityMonitor

def run_comparison_simulation():
    # --- SECTION 1: CONFIG ---
    dt = 0.05
    total_time = 25.0 
    steps = int(total_time / dt)
    
    # --- SECTION 2: INITIALIZATION ---
    path = Path(start_point=[0, 0], end_point=[100, 0])
    camera = CameraSensor(noise_std=0.05)
    monitor = ReliabilityMonitor(threshold=0.5) 
    
    # Resilient Robot (Uses Monitor)
    robot_res = BicycleRobot(x=0.0, y=2.0, theta=0.0, v=2.0, wheelbase=2.5)
    pid_res = PIDController(kp=0.4, ki=0.02, kd=0.5)
    
    # Naive Robot (Blindly trusts sensor)
    robot_naive = BicycleRobot(x=0.0, y=2.0, theta=0.0, v=2.0, wheelbase=2.5)
    pid_naive = PIDController(kp=0.4, ki=0.02, kd=0.5)

    # Histories for plotting
    x_res_hist, y_res_hist = [], []
    health_hist = []
    
    x_naive_hist, y_naive_hist = [], []
    sensor_naive_hist = [] # To visualize the glitches the naive robot acts upon

    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 6))

    # --- SECTION 3: THE LOOP ---
    for t in range(steps):
        current_time = t * dt
        
        # ==========================================
        #  NAIVE ROBOT PROCESSING
        # ==========================================
        # 1. Perception
        raw_camera_naive = camera.get_measurement(robot_naive.y, current_time)
        
        # 2. Control (No fault isolation!)
        cte_naive = path.get_cte([robot_naive.x, raw_camera_naive])
        steer_naive = pid_naive.control(cte_naive, dt)
        robot_naive.update(steering_angle=steer_naive, acceleration=0.0, dt=dt)
        
        # ==========================================
        #  RESILIENT ROBOT PROCESSING
        # ==========================================
        # A. PREDICTION (Shadow Model)
        predicted_y = robot_res.y 

        # B. PERCEPTION (The "Dirty" Camera)
        raw_camera_res = camera.get_measurement(robot_res.y, current_time)
        
        # C. RELIABILITY MONITOR
        is_healthy, residual = monitor.check_health(raw_camera_res, predicted_y)
        
        # D. FAULT ISOLATION
        input_y = raw_camera_res if is_healthy else predicted_y
        
        # E. CONTROL & PHYSICS
        cte_res = path.get_cte([robot_res.x, input_y])
        steer_res = pid_res.control(cte_res, dt)
        robot_res.update(steering_angle=steer_res, acceleration=0.0, dt=dt)

        # ==========================================
        #  LOGGING
        # ==========================================
        x_res_hist.append(robot_res.x)
        y_res_hist.append(robot_res.y)
        health_hist.append(is_healthy)
        
        x_naive_hist.append(robot_naive.x)
        y_naive_hist.append(robot_naive.y)
        sensor_naive_hist.append(raw_camera_naive)

        # --- SECTION 4: VISUALIZATION ---
        if t % 5 == 0: 
            ax.clear()
            # Reference Path
            ax.axhline(y=0, color='black', linestyle='--', alpha=0.5, label="Target Path")
            
            # Raw Sensor Data
            ax.scatter(x_naive_hist, sensor_naive_hist, color='red', s=4, alpha=0.2, label="Raw Sensor Glitches")
            
            # Robot Trajectories
            ax.plot(x_naive_hist, y_naive_hist, color='orange', linewidth=2, linestyle='--', label="Naive Robot (Fails)")
            ax.plot(x_res_hist, y_res_hist, color='blue', linewidth=2, label="Resilient Robot (Survives)")
            
            # Status Indicator
            status_color = "green" if is_healthy else "red"
            status_text = "SENSOR: HEALTHY" if is_healthy else "SENSOR: FAULT DETECTED!"
            
            # Dynamic camera following the robots
            max_x = max(robot_res.x, robot_naive.x)
            ax.text(max_x - 8, 3.5, status_text, color=status_color, fontweight='bold', fontsize=12)

            ax.set_ylim(-4, 5)
            ax.set_xlim(max(0, max_x - 15), max(20, max_x + 5))
            ax.set_title(f"OmniPath-DSP: Naive vs Resilient | Time: {current_time:.1f}s")
            ax.set_xlabel("X Position (m)")
            ax.set_ylabel("Y Position (m)")
            ax.legend(loc="lower left")
            plt.pause(0.001)

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    run_comparison_simulation()