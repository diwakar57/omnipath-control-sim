"""Basic simulation loop for OmniPath-DSP bicycle model."""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from models.bicycle_model import BicycleRobot


def run_simulation() -> None:
    dt = 0.05
    total_time = 10.0
    steps = int(total_time / dt)

    robot = BicycleRobot(x=0.0, y=0.0, theta=0.0, v=2.0, wheelbase=2.5)

    fixed_steering = np.deg2rad(20.0)
    acceleration = 0.0

    x_hist = [robot.x]
    y_hist = [robot.y]

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))

    for _ in range(steps):
        robot.update(steering_angle=fixed_steering, acceleration=acceleration, dt=dt)
        x_hist.append(robot.x)
        y_hist.append(robot.y)

        ax.clear()
        ax.plot(x_hist, y_hist, "b-", label="trajectory")
        ax.plot(robot.x, robot.y, "ro", label="robot")
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("Bicycle Model Circular Motion")
        ax.legend(loc="upper right")
        ax.grid(True)

        plt.pause(dt)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    run_simulation()
