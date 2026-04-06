"""Kinematic bicycle model for OmniPath-DSP."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class BicycleRobot:
    """Robot state and dynamics using a kinematic bicycle model."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    wheelbase: float = 2.5

    def update(self, steering_angle: float, acceleration: float, dt: float) -> None:
        """Advance state by one time step using Euler integration.

        Args:
            steering_angle: Front wheel steering angle (delta) in radians.
            acceleration: Longitudinal acceleration in m/s^2.
            dt: Time step in seconds.
        """
        if dt <= 0:
            raise ValueError("dt must be positive")

        max_steer = 0.6  # ~35 degrees
        steering_angle = np.clip(steering_angle, -max_steer, max_steer)
        
        x_dot = self.v * np.cos(self.theta)
        y_dot = self.v * np.sin(self.theta)
        theta_dot = (self.v / self.wheelbase) * np.tan(steering_angle)

        self.x += float(x_dot * dt)
        self.y += float(y_dot * dt)
        self.theta += float(theta_dot * dt)
        self.v += float(acceleration * dt)
