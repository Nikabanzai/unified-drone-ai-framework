"""
Unified controller: PID and RL modes with common interface.

Provides:
    - ControlMode: Enum for PID | RL | HYBRID
    - UnifiedController: Computes actions given observations and targets.

Design goals:
    - No imports from gym-pybullet-drones or crazyflow.
    - PID mode uses a simple cascaded position→attitude PID (pure numpy).
    - RL mode is a stub (loads a callable policy if provided).
    - Action output: attitude (4,) [roll, pitch, yaw, thrust].

Usage:
    from unified.control.controller import ControlMode, UnifiedController

    ctrl = UnifiedController(mode=ControlMode.PID)
    action = ctrl.compute(obs, target_pos=[0, 0, 1.0])
"""

from __future__ import annotations

import numpy as np
from enum import Enum
from typing import Callable, Optional, Dict, Any


class ControlMode(Enum):
    """Control mode selection."""

    PID = "pid"
    RL = "rl"
    HYBRID = "hybrid"


class UnifiedController:
    """
    Unified controller supporting PID (classical) and RL modes.

    Parameters
    ----------
    mode : ControlMode
        PID, RL, or HYBRID.
    drone_model : str
        Drone model name (affects mixer gains). Default "CF2X".
    policy : Callable, optional
        For RL mode: callable(obs) -> action (4,). If None, raises on compute.

    PID gains (tuned for Crazyflie-like dynamics):
        Position: kp=0.4, ki=0.05, kd=0.2
        Attitude: kp=70000, ki=0, kd=20000 (high for fast response)
    """

    def __init__(
        self,
        mode: ControlMode = ControlMode.PID,
        drone_model: str = "CF2X",
        policy: Optional[Callable[[Dict[str, np.ndarray]], np.ndarray]] = None,
    ):
        self.mode = mode
        self.drone_model = drone_model
        self.policy = policy

        # PID state (per-environment, single for now; extend for batch later)
        self._pos_err_i = np.zeros(3, dtype=np.float32)
        self._last_pos_err = np.zeros(3, dtype=np.float32)
        self._rpy_err_i = np.zeros(3, dtype=np.float32)
        self._last_rpy_err = np.zeros(3, dtype=np.float32)

        # PID gains (CF2X from DSLPIDControl)
        self.kp_pos = np.array([0.4, 0.4, 1.25], dtype=np.float32)
        self.ki_pos = np.array([0.05, 0.05, 0.05], dtype=np.float32)
        self.kd_pos = np.array([0.2, 0.2, 0.5], dtype=np.float32)

        self.kp_att = np.array([70000.0, 70000.0, 60000.0], dtype=np.float32)
        self.ki_att = np.array([0.0, 0.0, 500.0], dtype=np.float32)
        self.kd_att = np.array([20000.0, 20000.0, 12000.0], dtype=np.float32)

        self.g = 9.81  # gravity

    def reset(self) -> None:
        """Reset PID integrators."""
        self._pos_err_i[:] = 0.0
        self._last_pos_err[:] = 0.0
        self._rpy_err_i[:] = 0.0
        self._last_rpy_err[:] = 0.0

    def compute(
        self,
        obs: Dict[str, np.ndarray],
        target_pos: np.ndarray = None,
        target_rpy: np.ndarray = None,
        dt: float = 0.02,
    ) -> np.ndarray:
        """
        Compute control action.

        Parameters
        ----------
        obs : dict
            Observation dict with keys pos, quat, vel, ang_vel (each (3,) or (N,3)).
            For single drone, (3,)/(4,) arrays.
        target_pos : np.ndarray, optional
            Desired position (3,). Default [0,0,1].
        target_rpy : np.ndarray, optional
            Desired roll/pitch/yaw (3,). Default [0,0,0].
        dt : float
            Control timestep in seconds.

        Returns
        -------
        action : np.ndarray
            Shape (4,): [roll, pitch, yaw, thrust] in radians/Newtons.
        """
        if target_pos is None:
            target_pos = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        if target_rpy is None:
            target_rpy = np.zeros(3, dtype=np.float32)

        if self.mode == ControlMode.PID:
            return self._compute_pid(obs, target_pos, target_rpy, dt)

        elif self.mode == ControlMode.RL:
            if self.policy is None:
                raise RuntimeError("RL mode requires a policy callable")
            return self.policy(obs)

        elif self.mode == ControlMode.HYBRID:
            # Simple: use PID by default; RL hook for future
            return self._compute_pid(obs, target_pos, target_rpy, dt)

        else:
            raise ValueError(f"Unknown control mode: {self.mode}")

    def _compute_pid(
        self,
        obs: Dict[str, np.ndarray],
        target_pos: np.ndarray,
        target_rpy: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Cascaded position → attitude PID (single drone).

        Returns [roll, pitch, yaw, thrust] in radians/Newtons.
        """
        pos = np.asarray(obs["pos"], dtype=np.float32)
        quat = np.asarray(obs["quat"], dtype=np.float32)
        vel = np.asarray(obs["vel"], dtype=np.float32)

        # Position error
        pos_err = target_pos - pos

        # PID position → desired thrust vector
        self._pos_err_i += pos_err * dt
        pos_err_d = (pos_err - self._last_pos_err) / dt
        self._last_pos_err = pos_err.copy()

        target_thrust_vec = (
            self.kp_pos * pos_err + self.ki_pos * self._pos_err_i + self.kd_pos * pos_err_d
        )
        target_thrust_vec[2] += self.g  # gravity compensation (z up)

        # Thrust magnitude
        thrust = np.linalg.norm(target_thrust_vec)

        # Desired orientation: z-axis aligned with thrust vector
        z_des = target_thrust_vec / (thrust + 1e-9)
        yaw_des = target_rpy[2]

        # Construct desired rotation matrix (yaw + z_des)
        x_c = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0], dtype=np.float32)
        y_des = np.cross(z_des, x_c)
        y_des /= np.linalg.norm(y_des) + 1e-9
        x_des = np.cross(y_des, z_des)

        # Desired quaternion from rotation matrix (simple extraction)
        # For small angles, approximate roll/pitch from cross products
        roll = np.arctan2(y_des[2], z_des[2] + 1e-9)
        pitch = np.arctan2(-x_des[2], z_des[2] + 1e-9)

        # Attitude PID (optional: track target_rpy)
        rpy = _quat_to_rpy(quat)
        rpy_err = target_rpy - rpy
        self._rpy_err_i += rpy_err * dt
        rpy_err_d = (rpy_err - self._last_rpy_err) / dt
        self._last_rpy_err = rpy_err.copy()

        # Attitude correction (small angles)
        roll_corr = self.kp_att[0] * rpy_err[0] + self.kd_att[0] * rpy_err_d[0]
        pitch_corr = self.kp_att[1] * rpy_err[1] + self.kd_att[1] * rpy_err_d[1]
        yaw_corr = self.kp_att[2] * rpy_err[2] + self.ki_att[2] * self._rpy_err_i[2] + self.kd_att[2] * rpy_err_d[2]

        # Add corrections (scaled down for stability)
        roll += roll_corr * 1e-5
        pitch += pitch_corr * 1e-5
        yaw = yaw_des + yaw_corr * 1e-5

        return np.array([roll, pitch, yaw, thrust], dtype=np.float32)


def _quat_to_rpy(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion [x,y,z,w] to roll-pitch-yaw (radians)."""
    qx, qy, qz, qw = quat
    sinr = 2 * (qw * qx + qy * qz)
    cosr = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr, cosr)

    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny = 2 * (qw * qz + qx * qy)
    cosy = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny, cosy)

    return np.array([roll, pitch, yaw], dtype=np.float32)
