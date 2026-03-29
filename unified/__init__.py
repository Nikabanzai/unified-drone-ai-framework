"""
Unified Drone AI Framework

A unified framework combining:
    - RL (via stable-baselines3 / JAX policies)
    - Classical control (PID via gym-pybullet-drones)
    - Inspection planning (waypoints, coverage, collision check)

Built on:
    - crazyflow (JAX/MuJoCo simulation)
    - gym-pybullet-drones (Gymnasium + PID)

Quick start:
    from unified.adapters import gympybullet_to_crazyflow, rpm_to_attitude
    obs_cf = gympybullet_to_crazyflow(obs_gpb)
    action = rpm_to_attitude(rpms)
"""

__version__ = "0.1.0"
