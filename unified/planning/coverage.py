"""
Coverage path planning: boustrophedon grid planner.

Provides:
    - boustrophedon_grid(): Generate back-and-forth coverage waypoints.

Design goals:
    - Pure numpy, no external dependencies.
    - Output: list of (x, y, z) waypoints for inspection coverage.
    - Supports rectangular areas with configurable altitude and spacing.

Usage:
    from unified.planning.coverage import boustrophedon_grid

    waypoints = boustrophedon_grid(
        x_min=0, x_max=10, y_min=0, y_max=10,
        altitude=5.0, spacing=1.0
    )
"""

from __future__ import annotations

import numpy as np
from typing import List, Tuple


def boustrophedon_grid(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    altitude: float = 5.0,
    spacing: float = 1.0,
    start_corner: str = "bottom_left",
) -> List[Tuple[float, float, float]]:
    """
    Generate a boustrophedon (lawnmower) coverage path over a rectangle.

    Parameters
    ----------
    x_min, x_max : float
        X bounds of the rectangular area.
    y_min, y_max : float
        Y bounds of the rectangular area.
    altitude : float
        Constant Z altitude for all waypoints.
    spacing : float
        Distance between parallel sweep lines (meters).
    start_corner : str
        "bottom_left", "bottom_right", "top_left", "top_right".

    Returns
    -------
    waypoints : list of (x, y, z) tuples
        Ordered list of 3D waypoints.

    Notes
    -----
    Boustrophedon pattern: the drone sweeps back and forth across the area,
    like mowing a lawn. Each pass is offset by `spacing` in the perpendicular
    direction.
    """
    # Validate
    if x_max <= x_min or y_max <= y_min:
        raise ValueError("x_max > x_min and y_max > y_min required")
    if spacing <= 0:
        raise ValueError("spacing must be positive")

    # Determine sweep direction
    # Default: sweep along X, step along Y
    x_steps = np.arange(x_min, x_max + spacing * 0.5, spacing)
    y_steps = np.arange(y_min, y_max + spacing * 0.5, spacing)

    # Build waypoints
    waypoints: List[Tuple[float, float, float]] = []

    # Start corner determines initial direction
    if start_corner in ("bottom_left", "bottom_right"):
        y_order = list(y_steps)
    else:
        y_order = list(reversed(y_steps))

    sweep_forward = True
    for y in y_order:
        if sweep_forward:
            x_line = list(x_steps)
        else:
            x_line = list(reversed(x_steps))
        for x in x_line:
            waypoints.append((float(x), float(y), float(altitude)))
        sweep_forward = not sweep_forward

    return waypoints


def spiral_grid(
    x_center: float,
    y_center: float,
    radius: float,
    altitude: float = 5.0,
    spacing: float = 1.0,
    clockwise: bool = True,
) -> List[Tuple[float, float, float]]:
    """
    Generate a spiral coverage path (outward from center).

    Parameters
    ----------
    x_center, y_center : float
        Center of the spiral.
    radius : float
        Maximum radius of the spiral.
    altitude : float
        Constant Z altitude.
    spacing : float
        Radial spacing between spiral arms.
    clockwise : bool
        Spiral direction.

    Returns
    -------
    waypoints : list of (x, y, z) tuples
    """
    if radius <= 0 or spacing <= 0:
        raise ValueError("radius and spacing must be positive")

    waypoints: List[Tuple[float, float, float]] = []

    # Generate spiral using polar coordinates
    r = spacing
    theta = 0.0
    dtheta = 0.1  # angular step

    while r < radius:
        x = x_center + r * np.cos(theta)
        y = y_center + r * np.sin(theta)
        waypoints.append((float(x), float(y), float(altitude)))
        theta += dtheta
        # Increase radius slowly per full rotation
        if theta > 2 * np.pi:
            r += spacing
            theta -= 2 * np.pi

    # Ensure we close at center
    waypoints.append((float(x_center), float(y_center), float(altitude)))

    if not clockwise:
        waypoints.reverse()

    return waypoints
