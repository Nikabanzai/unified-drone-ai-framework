"""
State Adapter: Convert observations between crazyflow and gym-pybullet-drones formats.

This module provides pure-numpy conversion functions with no simulation imports.
Used to bridge the observation spaces of the two drone simulation frameworks.

gym-pybullet-drones observation (CtrlAviary._computeObs via _getDroneStateVector):
    Shape: (20,) per drone
    Contents:
        [0:3]   = X, Y, Z (position in meters)
        [3:7]   = Q1, Q2, Q3, Q4 (quaternion [x, y, z, w] from scipy Rotation)
        [7:10]  = R, P, Y (roll, pitch, yaw in radians)
        [10:13] = VX, VY, VZ (velocity in m/s)
        [13:16] = WX, WY, WZ (angular velocity in rad/s)
        [16:20] = P0, P1, P2, P3 (motor RPMs)

crazyflow observation (DroneEnv.single_observation_space):
    Dict with keys:
        "pos":     (3,) - position [x, y, z] in meters
        "quat":    (4,) - quaternion [x, y, z, w]
        "vel":     (3,) - velocity [vx, vy, vz] in m/s
        "ang_vel": (3,) - angular velocity [wx, wy, wz] in rad/s

Note: crazyflow does NOT include RPY or RPMs; only position/attitude/velocity.
"""

import numpy as np
from typing import Dict, Tuple, Optional


# gym-pybullet-drones observation indices (CtrlAviary._getDroneStateVector)
# See BaseAviary._getDroneStateVector in gym_pybullet_drones/envs/BaseAviary.py
GPB_IDX = {
    "pos": slice(0, 3),
    "quat": slice(3, 7),
    "rpy": slice(7, 10),
    "vel": slice(10, 13),
    "ang_vel": slice(13, 16),
    "rpm": slice(16, 20),
}

# crazyflow observation dict keys (DroneEnv.single_observation_space)
CF_KEYS = ("pos", "quat", "vel", "ang_vel")


def gympybullet_to_crazyflow(obs: np.ndarray) -> Dict[str, np.ndarray]:
    """
    Convert a gym-pybullet-drones observation (20,) to crazyflow observation dict.

    Parameters
    ----------
    obs : np.ndarray
        Shape (20,) or (NUM_DRONES, 20). Single-drone or batched observation.
        If batched, returns a dict of arrays with leading batch dimension.

    Returns
    -------
    dict
        Crazyflow-style observation:
            - "pos": (3,) or (N, 3)
            - "quat": (4,) or (N, 4)
            - "vel": (3,) or (N, 3)
            - "ang_vel": (3,) or (N, 3)

    Notes
    -----
    - RPY and RPMs from gym-pybullet-drones are discarded (not in crazyflow obs).
    - Quaternion convention: [x, y, z, w] (matches scipy Rotation.as_quat()).
    """
    obs = np.asarray(obs, dtype=np.float32)

    if obs.ndim == 1:
        # Single drone: (20,)
        if obs.shape != (20,):
            raise ValueError(f"Expected shape (20,), got {obs.shape}")
        return {
            "pos": obs[GPB_IDX["pos"]].copy(),
            "quat": obs[GPB_IDX["quat"]].copy(),
            "vel": obs[GPB_IDX["vel"]].copy(),
            "ang_vel": obs[GPB_IDX["ang_vel"]].copy(),
        }

    elif obs.ndim == 2:
        # Batched: (NUM_DRONES, 20)
        num_drones = obs.shape[0]
        if obs.shape[1] != 20:
            raise ValueError(f"Expected shape (N, 20), got {obs.shape}")
        return {
            "pos": obs[:, GPB_IDX["pos"]].copy(),
            "quat": obs[:, GPB_IDX["quat"]].copy(),
            "vel": obs[:, GPB_IDX["vel"]].copy(),
            "ang_vel": obs[:, GPB_IDX["ang_vel"]].copy(),
        }

    else:
        raise ValueError(f"Expected 1D (20,) or 2D (N, 20) array, got {obs.ndim}D")


def crazyflow_to_gympybullet(
    obs: Dict[str, np.ndarray],
    rpm: Optional[np.ndarray] = None,
) -> np.ndarray:
    """
    Convert a crazyflow observation dict to gym-pybullet-drones observation (20,).

    Parameters
    ----------
    obs : dict
        Crazyflow observation with keys: "pos", "quat", "vel", "ang_vel".
        Values can be (3,)/(4,) for single drone or (N, 3)/(N, 4) for batched.
    rpm : np.ndarray, optional
        Motor RPMs to include. If None, zeros are used.
        Shape: (4,) for single drone, (N, 4) for batched.

    Returns
    -------
    np.ndarray
        Shape (20,) for single drone, (N, 20) for batched.

    Notes
    -----
    - RPY (indices 7-9) is computed from quaternion via _quat_to_rpy().
    - If rpm is not provided, motor RPMs (indices 16-19) are set to zero.
    - Quaternion must be [x, y, z, w] convention.
    """
    # Validate keys
    for key in CF_KEYS:
        if key not in obs:
            raise KeyError(f"Missing key '{key}' in obs dict")

    pos = np.asarray(obs["pos"], dtype=np.float32)
    quat = np.asarray(obs["quat"], dtype=np.float32)
    vel = np.asarray(obs["vel"], dtype=np.float32)
    ang_vel = np.asarray(obs["ang_vel"], dtype=np.float32)

    # Determine single vs batched
    if pos.ndim == 1:
        # Single drone
        if pos.shape != (3,):
            raise ValueError(f"pos must be (3,), got {pos.shape}")
        if quat.shape != (4,):
            raise ValueError(f"quat must be (4,), got {quat.shape}")
        if vel.shape != (3,):
            raise ValueError(f"vel must be (3,), got {vel.shape}")
        if ang_vel.shape != (3,):
            raise ValueError(f"ang_vel must be (3,), got {ang_vel.shape}")

        out = np.zeros(20, dtype=np.float32)
        out[GPB_IDX["pos"]] = pos
        out[GPB_IDX["quat"]] = quat
        out[GPB_IDX["rpy"]] = _quat_to_rpy(quat)
        out[GPB_IDX["vel"]] = vel
        out[GPB_IDX["ang_vel"]] = ang_vel

        if rpm is not None:
            rpm = np.asarray(rpm, dtype=np.float32)
            if rpm.shape != (4,):
                raise ValueError(f"rpm must be (4,), got {rpm.shape}")
            out[GPB_IDX["rpm"]] = rpm

        return out

    elif pos.ndim == 2:
        # Batched
        num_drones = pos.shape[0]
        if quat.shape != (num_drones, 4):
            raise ValueError(f"quat must be (N, 4), got {quat.shape}")
        if vel.shape != (num_drones, 3):
            raise ValueError(f"vel must be (N, 3), got {vel.shape}")
        if ang_vel.shape != (num_drones, 3):
            raise ValueError(f"ang_vel must be (N, 3), got {ang_vel.shape}")

        out = np.zeros((num_drones, 20), dtype=np.float32)
        out[:, GPB_IDX["pos"]] = pos
        out[:, GPB_IDX["quat"]] = quat
        out[:, GPB_IDX["rpy"]] = np.stack([_quat_to_rpy(q) for q in quat])
        out[:, GPB_IDX["vel"]] = vel
        out[:, GPB_IDX["ang_vel"]] = ang_vel

        if rpm is not None:
            rpm = np.asarray(rpm, dtype=np.float32)
            if rpm.shape != (num_drones, 4):
                raise ValueError(f"rpm must be (N, 4), got {rpm.shape}")
            out[:, GPB_IDX["rpm"]] = rpm

        return out

    else:
        raise ValueError(f"pos must be 1D or 2D, got {pos.ndim}D")


def _quat_to_rpy(quat: np.ndarray) -> np.ndarray:
    """
    Convert quaternion [x, y, z, w] to roll-pitch-yaw (radians).

    Uses the standard aerospace convention (Z-Y-X Euler angles).
    Pure numpy implementation, no external dependencies.

    Parameters
    ----------
    quat : np.ndarray
        Quaternion as [qx, qy, qz, qw], shape (4,).

    Returns
    -------
    np.ndarray
        [roll, pitch, yaw] in radians, shape (3,).
    """
    qx, qy, qz, qw = quat

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # clamp to ±90°
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float32)
