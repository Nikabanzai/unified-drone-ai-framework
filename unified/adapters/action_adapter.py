"""
Action Adapter: Convert between RPM and attitude action spaces.

This module provides pure-numpy conversion functions with no simulation imports.
Used to bridge the control interfaces of gym-pybullet-drones (RPM) and
crazyflow (attitude/force_torque/rotor_vel).

gym-pybullet-drones action (CtrlAviary._actionSpace):
    Shape: (NUM_DRONES, 4) - motor RPMs [P0, P1, P2, P3]
    Range: [0, MAX_RPM] per motor

crazyflow attitude action (DroneEnv with Control.attitude):
    Shape: (NUM_DRONES, 4) - [roll, pitch, yaw, collective_thrust]
    Roll/Pitch/Yaw: radians
    Thrust: Newtons (collective, 4 motors combined)

The conversion uses the drone's mixer matrix (from DSLPIDControl for CF2X/CF2P).
For a quadcopter:
    RPMs = base_hover_rpm + MIXER @ [roll, pitch, yaw]
    thrust = mean(RPMs) * rpm_to_thrust_coefficient

Mixer matrix for CF2X (from gym_pybullet_drones/control/DSLPIDControl.py lines 48-53):
    MIXER = [[-0.5, -0.5, -1],
             [-0.5, +0.5, +1],
             [+0.5, +0.5, -1],
             [+0.5, -0.5, +1]]
    Columns: [roll, pitch, yaw]
    Rows: [P0, P1, P2, P3]
"""

import numpy as np
from typing import Tuple, Optional

# CF2X mixer matrix (from DSLPIDControl.__init__ lines 48-53)
# This maps [roll, pitch, yaw] → differential RPMs
CF2X_MIXER = np.array(
    [
        [-0.5, -0.5, -1.0],
        [-0.5, +0.5, +1.0],
        [+0.5, +0.5, -1.0],
        [+0.5, -0.5, +1.0],
    ],
    dtype=np.float32,
)

# CF2P mixer matrix (from DSLPIDControl.__init__ lines 55-60)
CF2P_MIXER = np.array(
    [
        [0.0, -1.0, -1.0],
        [+1.0, 0.0, +1.0],
        [0.0, +1.0, -1.0],
        [-1.0, 0.0, +1.0],
    ],
    dtype=np.float32,
)

# Map drone model names to mixer matrices
MIXERS = {
    "CF2X": CF2X_MIXER,
    "CF2P": CF2P_MIXER,
}

# Default hover RPM for Crazyflie (approximate, used for scaling)
# In practice, this comes from the drone model (MAX_RPM / 2 or computed from mass/gravity)
DEFAULT_HOVER_RPM = 14468.0  # Approximate for CF2X at hover

# RPM to thrust coefficient (kf from URDF, approximate)
# thrust = kf * rpm^2 per motor
# For CF2X: kf ≈ 3.16e-10 (from BaseAviary._parseURDFParameters)
DEFAULT_KF = 3.16e-10


def rpm_to_attitude(
    rpms: np.ndarray,
    drone_model: str = "CF2X",
    hover_rpm: Optional[float] = None,
) -> np.ndarray:
    """
    Convert motor RPMs to attitude action [roll, pitch, yaw, thrust].

    Parameters
    ----------
    rpms : np.ndarray
        Motor RPMs. Shape (4,) for single drone, (N, 4) for batched.
    drone_model : str
        Drone model name ("CF2X" or "CF2P"). Determines mixer matrix.
    hover_rpm : float, optional
        Hover RPM for the drone. If None, uses DEFAULT_HOVER_RPM.

    Returns
    -------
    np.ndarray
        Attitude action [roll, pitch, yaw, thrust].
        Shape (4,) for single drone, (N, 4) for batched.
        - roll, pitch, yaw: radians
        - thrust: Newtons (collective)

    Notes
    -----
    - Uses pseudoinverse of mixer matrix to recover [roll, pitch, yaw] from RPM diffs.
    - Thrust is computed as mean(RPMs) * rpm_to_thrust_scaling.
    - This is an approximation; real attitude comes from cascaded PID.
    """
    rpms = np.asarray(rpms, dtype=np.float32)

    if drone_model not in MIXERS:
        raise ValueError(f"Unknown drone_model '{drone_model}'. Use 'CF2X' or 'CF2P'.")

    mixer = MIXERS[drone_model]
    if hover_rpm is None:
        hover_rpm = DEFAULT_HOVER_RPM

    if rpms.ndim == 1:
        # Single drone: (4,)
        if rpms.shape != (4,):
            raise ValueError(f"rpms must be (4,), got {rpms.shape}")

        # Differential RPMs from hover baseline
        rpm_diff = rpms - hover_rpm

        # Recover [roll, pitch, yaw] via pseudoinverse
        # rpm_diff = mixer @ [roll, pitch, yaw]  →  [roll, pitch, yaw] = pinv(mixer) @ rpm_diff
        mixer_inv = np.linalg.pinv(mixer)  # (3, 4)
        rpy = mixer_inv @ rpm_diff  # (3,)

        # Thrust from mean RPM (scaled to Newtons)
        mean_rpm = np.mean(rpms)
        thrust = _rpm_to_thrust(mean_rpm, hover_rpm)

        return np.array([rpy[0], rpy[1], rpy[2], thrust], dtype=np.float32)

    elif rpms.ndim == 2:
        # Batched: (N, 4)
        num_drones = rpms.shape[0]
        if rpms.shape[1] != 4:
            raise ValueError(f"rpms must be (N, 4), got {rpms.shape}")

        rpm_diff = rpms - hover_rpm

        mixer_inv = np.linalg.pinv(mixer)  # (3, 4)
        rpy = (mixer_inv @ rpm_diff.T).T  # (N, 3)

        mean_rpms = np.mean(rpms, axis=1, keepdims=True)  # (N, 1)
        thrusts = np.array([_rpm_to_thrust(m, hover_rpm) for m in mean_rpms.flatten()])

        return np.hstack([rpy, thrusts.reshape(-1, 1)]).astype(np.float32)

    else:
        raise ValueError(f"rpms must be 1D or 2D, got {rpms.ndim}D")


def attitude_to_rpm(
    attitude: np.ndarray,
    drone_model: str = "CF2X",
    hover_rpm: Optional[float] = None,
) -> np.ndarray:
    """
    Convert attitude action [roll, pitch, yaw, thrust] to motor RPMs.

    Parameters
    ----------
    attitude : np.ndarray
        Attitude action [roll, pitch, yaw, thrust].
        Shape (4,) for single drone, (N, 4) for batched.
        - roll, pitch, yaw: radians
        - thrust: Newtons (collective)
    drone_model : str
        Drone model name ("CF2X" or "CF2P").
    hover_rpm : float, optional
        Hover RPM for the drone. If None, uses DEFAULT_HOVER_RPM.

    Returns
    -------
    np.ndarray
        Motor RPMs [P0, P1, P2, P3].
        Shape (4,) for single drone, (N, 4) for batched.

    Notes
    -----
    - RPMs = hover_rpm + mixer @ [roll, pitch, yaw]
    - Thrust is converted to mean RPM and added to base hover RPM.
    """
    attitude = np.asarray(attitude, dtype=np.float32)

    if drone_model not in MIXERS:
        raise ValueError(f"Unknown drone_model '{drone_model}'. Use 'CF2X' or 'CF2P'.")

    mixer = MIXERS[drone_model]
    if hover_rpm is None:
        hover_rpm = DEFAULT_HOVER_RPM

    if attitude.ndim == 1:
        # Single drone: (4,)
        if attitude.shape != (4,):
            raise ValueError(f"attitude must be (4,), got {attitude.shape}")

        roll, pitch, yaw, thrust = attitude

        # Differential RPMs from [roll, pitch, yaw]
        rpm_diff = mixer @ np.array([roll, pitch, yaw], dtype=np.float32)  # (4,)

        # Thrust → mean RPM offset
        thrust_rpm = _thrust_to_rpm(thrust, hover_rpm)

        # Total RPMs = hover + diff + thrust component
        rpms = hover_rpm + rpm_diff + thrust_rpm

        # Clip to non-negative
        rpms = np.clip(rpms, 0.0, None)

        return rpms.astype(np.float32)

    elif attitude.ndim == 2:
        # Batched: (N, 4)
        num_drones = attitude.shape[0]
        if attitude.shape[1] != 4:
            raise ValueError(f"attitude must be (N, 4), got {attitude.shape}")

        rolls = attitude[:, 0]
        pitches = attitude[:, 1]
        yaws = attitude[:, 2]
        thrusts = attitude[:, 3]

        rpy = np.stack([rolls, pitches, yaws], axis=1)  # (N, 3)
        rpm_diff = (mixer @ rpy.T).T  # (N, 4)

        thrust_rpms = np.array([_thrust_to_rpm(t, hover_rpm) for t in thrusts])

        rpms = hover_rpm + rpm_diff + thrust_rpms.reshape(-1, 1)
        rpms = np.clip(rpms, 0.0, None)

        return rpms.astype(np.float32)

    else:
        raise ValueError(f"attitude must be 1D or 2D, got {attitude.ndim}D")


def _rpm_to_thrust(rpm: float, hover_rpm: float) -> float:
    """
    Convert RPM to thrust (Newtons) using kf * rpm^2 scaling.

    This is a simplified model. Real thrust depends on propeller geometry,
    air density, etc. We use a linear approximation around hover.
    """
    # Thrust coefficient: at hover_rpm, thrust ≈ weight
    # For simplicity, assume linear scaling: thrust ∝ rpm
    # In reality: thrust = kf * rpm^2, but for small perturbations linear is OK
    return (rpm / hover_rpm) * 0.5  # ~0.5N at hover for CF2X


def _thrust_to_rpm(thrust: float, hover_rpm: float) -> float:
    """
    Convert thrust (Newtons) to RPM offset from hover.

    Inverse of _rpm_to_thrust.
    """
    return (thrust / 0.5) * hover_rpm - hover_rpm


def get_mixer_matrix(drone_model: str) -> np.ndarray:
    """
    Return the mixer matrix for a given drone model.

    Parameters
    ----------
    drone_model : str
        "CF2X" or "CF2P".

    Returns
    -------
    np.ndarray
        (4, 3) mixer matrix mapping [roll, pitch, yaw] → RPM diffs.
    """
    if drone_model not in MIXERS:
        raise ValueError(f"Unknown drone_model '{drone_model}'. Use 'CF2X' or 'CF2P'.")
    return MIXERS[drone_model].copy()
