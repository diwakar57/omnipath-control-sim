"""Path generation utilities."""

from __future__ import annotations

import numpy as np


class Path:
    def __init__(self, start_point, end_point):
        self.start = np.array(start_point)

    def get_closest_point(self, robot_pos):
        # For a straight horizontal line at y = start_y
        # The goal is simply the projection of the robot onto the line
        return np.array([robot_pos[0], self.start[1]])

    def get_cte(self, robot_pos):
        # Cross-Track Error: Vertical distance from the line y = start_y
        return robot_pos[1] - self.start[1]
