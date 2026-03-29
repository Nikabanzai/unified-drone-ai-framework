"""
Mission orchestrator: wires planning → control → simulation.

Provides:
    - MissionOrchestrator: Execute a waypoint plan using a controller and simulator.

Design goals:
    - No imports from crazyflow or gym-pybullet-drones.
    - Uses SimulationBackend ABC (works with MockBackend).
    - Uses UnifiedController (PID mode by default).
    - Pure numpy; mock-friendly.

Usage:
    from unified.mission.orchestrator import MissionOrchestrator
    from unified.simulation.backend import MockBackend

    orch = MissionOrchestrator(sim=MockBackend(num_envs=1))
    orch.load_plan([(0,0,1), (1,0,1), (1,1,1)])
    orch.run(max_steps=50)
"""

from __future__ import annotations

import numpy as np
from typing import List, Tuple, Optional

from unified.simulation.backend import SimulationBackend, MockBackend
from unified.control.controller import ControlMode, UnifiedController


class MissionOrchestrator:
    """
    Execute a waypoint mission using controller + simulator.

    Parameters
    ----------
    sim : SimulationBackend
        Simulation backend (use MockBackend for testing).
    controller : UnifiedController, optional
        If None, creates default PID controller.
    """

    def __init__(
        self,
        sim: Optional[SimulationBackend] = None,
        controller: Optional[UnifiedController] = None,
    ):
        self.sim = sim if sim is not None else MockBackend(num_envs=1)
        self.controller = controller if controller is not None else UnifiedController(mode=ControlMode.PID)
        self.waypoints: List[Tuple[float, float, float]] = []
        self.current_wp_idx = 0
        self._log: List[dict] = []

    def load_plan(self, waypoints: List[Tuple[float, float, float]]) -> None:
        """Load a list of (x, y, z) waypoints."""
        self.waypoints = list(waypoints)
        self.current_wp_idx = 0
        self._log.clear()

    def reset(self) -> None:
        """Reset simulator and controller."""
        self.sim.reset()
        self.controller.reset()
        self.current_wp_idx = 0

    def run(
        self,
        max_steps: int = 1000,
        waypoint_tolerance: float = 0.5,
    ) -> List[dict]:
        """
        Run the mission until all waypoints reached or max_steps hit.

        Returns
        -------
        log : list of dict
            Per-step telemetry: {step, wp_idx, pos, action}
        """
        self.reset()
        log: List[dict] = []

        for step in range(max_steps):
            if self.current_wp_idx >= len(self.waypoints):
                break  # mission complete

            target = np.array(self.waypoints[self.current_wp_idx], dtype=np.float32)
            obs = self.sim.get_obs()

            # Extract single-drone obs (first env)
            obs_single = {k: v[0] for k, v in obs.items()}

            action = self.controller.compute(obs_single, target_pos=target)

            # Step sim (action is (4,) → repeat for num_envs)
            action_batch = np.tile(action, (self.sim.num_envs, 1))
            obs_new, reward, done, info = self.sim.step(action_batch)

            # Check waypoint reached
            pos = obs_new["pos"][0]
            dist = np.linalg.norm(pos - target)
            if dist < waypoint_tolerance:
                self.current_wp_idx += 1

            # Log
            entry = {
                "step": step,
                "wp_idx": self.current_wp_idx,
                "pos": pos.tolist(),
                "target": target.tolist(),
                "dist": float(dist),
            }
            log.append(entry)
            self._log.append(entry)

        return log

    @property
    def progress(self) -> float:
        """Fraction of waypoints completed [0.0, 1.0]."""
        if not self.waypoints:
            return 1.0
        return self.current_wp_idx / len(self.waypoints)
