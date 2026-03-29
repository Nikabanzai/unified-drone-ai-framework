"""
Inspection planning module.

Provides waypoint generation, coverage path planning, and collision checking
for drone inspection missions.

Submodules:
    - coverage.py: boustrophedon_grid(), spiral_grid()
"""

from .coverage import boustrophedon_grid, spiral_grid

__all__ = ["boustrophedon_grid", "spiral_grid"]
