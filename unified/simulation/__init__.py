"""
Simulation backends for the unified framework.

Provides a unified interface to swap between crazyflow (JAX/MuJoCo) and
gym-pybullet-drones (PyBullet) as the physics simulation backend.

Usage (future):
    from unified.simulation import Backend, UnifiedSimulator

    sim = UnifiedSimulator(backend=Backend.CRAZYFLOW, num_envs=4)
    obs = sim.reset()
    obs, reward, done, info = sim.step(action)
"""

from enum import Enum
from .backend import SimulationBackend, MockBackend

__all__ = ["Backend", "SimulationBackend", "MockBackend"]


class Backend(Enum):
    """Simulation backend selection."""

    CRAZYFLOW = "crazyflow"  # JAX + MuJoCo (primary, differentiable)
    PYBULLET = "pybullet"    # gym-pybullet-drones (secondary, visual debug)
