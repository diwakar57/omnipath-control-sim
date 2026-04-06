"""Path generation utilities."""

from __future__ import annotations

import numpy as np


class Path:
    """Straight horizontal path utility built from start and end points.

    Notes:
        `end_point` is retained to keep a consistent path interface and allow
        future extension to non-horizontal line handling without API changes.
    """

    def __init__(
        self,
        start_point: np.ndarray | list[float] | tuple[float, float],
        end_point: np.ndarray | list[float] | tuple[float, float],
    ) -> None:
        """Initialize a straight path using array-like [x, y] endpoints."""
        self.start = np.array(start_point)
        self.end = np.array(end_point)

    def get_closest_point(
        self, robot_pos: np.ndarray | list[float] | tuple[float, float]
    ) -> np.ndarray:
        """Return closest point [x, y] on horizontal line y=start_y for robot_pos."""
        if len(robot_pos) < 2:
            raise ValueError("robot_pos must have at least two coordinates")
        # For a straight horizontal line at y = start_y
        # The goal is simply the projection of the robot onto the line
        return np.array([robot_pos[0], self.start[1]])

    def get_cte(
        self, robot_pos: np.ndarray | list[float] | tuple[float, float]
    ) -> float:
        """Return signed cross-track error; positive means robot is above the line."""
        if len(robot_pos) < 2:
            raise ValueError("robot_pos must have at least two coordinates")
        # Cross-Track Error: Vertical distance from the line y = start_y
        return float(robot_pos[1] - self.start[1])
