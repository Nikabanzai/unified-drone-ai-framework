"""
Control layer: PID and RL controllers with unified interface.

Usage (future):
    from unified.control import ControlMode, UnifiedController

    ctrl = UnifiedController(mode=ControlMode.PID)
    action = ctrl.compute(observation, target)
"""

from enum import Enum
from .controller import UnifiedController

__all__ = ["ControlMode", "UnifiedController"]


class ControlMode(Enum):
    """Control mode selection for the unified controller."""

    PID = "pid"       # Classical PID (DSLPIDControl from gym-pybullet-drones)
    RL = "rl"         # RL policy (stable-baselines3 PPO or JAX policy)
    HYBRID = "hybrid" # Switch between PID and RL per mission phase
