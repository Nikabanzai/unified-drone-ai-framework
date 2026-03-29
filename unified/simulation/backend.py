"""
Simulation backend abstraction and mock implementation.

Provides:
    - SimulationBackend: Abstract base class defining the simulation interface.
    - MockBackend: Pure-numpy mock for testing without real physics.

Design goals:
    - No imports from crazyflow or gym-pybullet-drones.
    - All backends expose the same interface (reset, step, get_obs, close).
    - State is a dict with keys matching crazyflow: {"pos", "quat", "vel", "ang_vel"}.
    - Action is attitude (4,) or RPM (4,) depending on control mode.

Usage:
    from unified.simulation.backend import SimulationBackend, MockBackend

    sim = MockBackend(num_envs=2)
    obs = sim.reset()
    obs, reward, done, info = sim.step(action)  # action shape (2, 4)
"""

from __future__ import annotations

import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, Optional


class SimulationBackend(ABC):
    """
    Abstract base class for simulation backends.

    All backends must implement:
        - reset() -> Dict[str, np.ndarray]
        - step(action) -> (obs, reward, done, info)
        - get_obs() -> Dict[str, np.ndarray]
        - close() -> None

    State/observation dict keys (matches crazyflow DroneEnv):
        - "pos": (N, 3) position in meters
        - "quat": (N, 4) quaternion [x, y, z, w]
        - "vel": (N, 3) velocity in m/s
        - "ang_vel": (N, 3) angular velocity in rad/s

    Action shapes:
        - attitude: (N, 4) [roll, pitch, yaw, thrust]
        - rpm: (N, 4) motor RPMs
    """

    def __init__(self, num_envs: int = 1, **kwargs):
        self.num_envs = num_envs
        self._step_count = 0

    @abstractmethod
    def reset(self, seed: Optional[int] = None, **kwargs) -> Dict[str, np.ndarray]:
        """Reset all environments and return initial observation dict."""
        pass

    @abstractmethod
    def step(
        self, action: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray, Dict[str, Any]]:
        """
        Advance simulation by one step.

        Parameters
        ----------
        action : np.ndarray
            Shape (N, 4). Action type (attitude/RPM) depends on backend config.

        Returns
        -------
        obs : dict
            Observation dict with keys pos, quat, vel, ang_vel (each (N, D)).
        reward : np.ndarray
            Shape (N,). Scalar reward per environment.
        done : np.ndarray
            Shape (N,). Boolean termination flag.
        info : dict
            Additional info (empty dict for mock).
        """
        pass

    @abstractmethod
    def get_obs(self) -> Dict[str, np.ndarray]:
        """Return current observation without stepping."""
        pass

    def close(self) -> None:
        """Cleanup resources (no-op for mock)."""
        pass

    @property
    def step_count(self) -> int:
        """Number of steps taken since last reset."""
        return self._step_count


class MockBackend(SimulationBackend):
    """
    Pure-numpy mock simulation backend for testing.

    - reset() initializes drones at origin with identity quaternion.
    - step() performs a trivial integration (pos += vel * dt, etc.).
    - No actual physics — just enough to exercise the interface.

    Parameters
    ----------
    num_envs : int
        Number of parallel environments.
    dt : float
        Time step in seconds (default 0.02 = 50 Hz).
    """

    def __init__(self, num_envs: int = 1, dt: float = 0.02, **kwargs):
        super().__init__(num_envs=num_envs, **kwargs)
        self.dt = dt
        self._pos = np.zeros((num_envs, 3), dtype=np.float32)
        self._quat = np.zeros((num_envs, 4), dtype=np.float32)
        self._quat[:, 3] = 1.0  # identity quaternion [0,0,0,1]
        self._vel = np.zeros((num_envs, 3), dtype=np.float32)
        self._ang_vel = np.zeros((num_envs, 3), dtype=np.float32)

    def reset(self, seed: Optional[int] = None, **kwargs) -> Dict[str, np.ndarray]:
        if seed is not None:
            np.random.seed(seed)
        self._pos[:] = 0.0
        self._quat[:] = 0.0
        self._quat[:, 3] = 1.0
        self._vel[:] = 0.0
        self._ang_vel[:] = 0.0
        self._step_count = 0
        return self.get_obs()

    def step(
        self, action: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray, Dict[str, Any]]:
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (self.num_envs, 4):
            raise ValueError(f"Action must be ({self.num_envs}, 4), got {action.shape}")

        # Trivial "physics": thrust in z increases altitude
        thrust = action[:, 3]  # (N,)
        self._vel[:, 2] += thrust * self.dt * 0.1  # crude acceleration
        self._pos += self._vel * self.dt

        # Clamp to simple bounds
        self._pos = np.clip(self._pos, -100, 100)

        self._step_count += 1

        obs = self.get_obs()
        reward = np.zeros(self.num_envs, dtype=np.float32)
        done = np.zeros(self.num_envs, dtype=bool)

        return obs, reward, done, {}

    def get_obs(self) -> Dict[str, np.ndarray]:
        return {
            "pos": self._pos.copy(),
            "quat": self._quat.copy(),
            "vel": self._vel.copy(),
            "ang_vel": self._ang_vel.copy(),
        }
