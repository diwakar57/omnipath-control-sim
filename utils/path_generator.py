"""Path generation utilities."""

from __future__ import annotations

import numpy as np


class Path:
    """Straight horizontal path utility built from start and end points."""

    def __init__(self, start_point, end_point):
        """Initialize a straight path using array-like [x, y] endpoints."""
        self.start = np.array(start_point)
        self.end = np.array(end_point)

    def get_closest_point(self, robot_pos):
        """Return closest point [x, y] on horizontal line y=start_y for robot_pos."""
        # For a straight horizontal line at y = start_y
        # The goal is simply the projection of the robot onto the line
        return np.array([robot_pos[0], self.start[1]])

    def get_cte(self, robot_pos):
        """Return signed cross-track error; positive means robot is above the line."""
        # Cross-Track Error: Vertical distance from the line y = start_y
        return robot_pos[1] - self.start[1]
