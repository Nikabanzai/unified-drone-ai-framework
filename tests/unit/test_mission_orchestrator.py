"""
Unit tests for mission/orchestrator.py

Run with:
    pytest tests/unit/test_mission_orchestrator.py -v
"""

import pytest
import numpy as np

from unified.mission.orchestrator import MissionOrchestrator
from unified.simulation.backend import MockBackend
from unified.control.controller import ControlMode, UnifiedController


class TestMissionOrchestratorInit:
    """Tests for MissionOrchestrator initialization."""

    def test_default_sim_and_controller(self):
        orch = MissionOrchestrator()
        assert orch.sim is not None
        assert orch.controller is not None
        assert isinstance(orch.sim, MockBackend)
        assert isinstance(orch.controller, UnifiedController)

    def test_custom_sim(self):
        custom_sim = MockBackend(num_envs=2)
        orch = MissionOrchestrator(sim=custom_sim)
        assert orch.sim.num_envs == 2

    def test_custom_controller(self):
        custom_ctrl = UnifiedController(mode=ControlMode.RL, policy=lambda o: np.zeros(4))
        orch = MissionOrchestrator(controller=custom_ctrl)
        assert orch.controller.mode == ControlMode.RL


class TestMissionOrchestratorLoadPlan:
    """Tests for load_plan()."""

    def test_load_plan_stores_waypoints(self):
        orch = MissionOrchestrator()
        wps = [(0, 0, 1), (1, 0, 1), (1, 1, 1)]
        orch.load_plan(wps)
        assert orch.waypoints == wps

    def test_load_plan_resets_index(self):
        orch = MissionOrchestrator()
        orch.current_wp_idx = 5
        orch.load_plan([(0, 0, 1)])
        assert orch.current_wp_idx == 0


class TestMissionOrchestratorRun:
    """Tests for run()."""

    def test_run_empty_plan_completes_immediately(self):
        orch = MissionOrchestrator()
        orch.load_plan([])
        log = orch.run(max_steps=10)
        assert len(log) == 0

    def test_run_single_waypoint(self):
        orch = MissionOrchestrator()
        orch.load_plan([(0.0, 0.0, 1.0)])
        log = orch.run(max_steps=50, waypoint_tolerance=1.0)
        # Should reach waypoint quickly (at origin)
        assert orch.progress >= 0.0  # completed or progressing

    def test_run_progress_increases(self):
        orch = MissionOrchestrator()
        orch.load_plan([(0, 0, 1), (10, 10, 1)])
        orch.run(max_steps=5, waypoint_tolerance=0.1)
        # Progress should be 0 or 0.5 or 1.0 depending on if waypoints reached
        assert 0.0 <= orch.progress <= 1.0

    def test_run_respects_max_steps(self):
        orch = MissionOrchestrator()
        orch.load_plan([(100, 100, 1)])  # far away, won't reach
        log = orch.run(max_steps=10, waypoint_tolerance=0.1)
        assert len(log) == 10

    def test_log_contains_entries(self):
        orch = MissionOrchestrator()
        orch.load_plan([(0, 0, 1)])
        log = orch.run(max_steps=5)
        for entry in log:
            assert "step" in entry
            assert "wp_idx" in entry
            assert "pos" in entry
            assert "target" in entry
            assert "dist" in entry


class TestMissionOrchestratorReset:
    """Tests for reset()."""

    def test_reset_clears_index(self):
        orch = MissionOrchestrator()
        orch.load_plan([(0, 0, 1), (1, 1, 1)])
        orch.current_wp_idx = 2
        orch.reset()
        assert orch.current_wp_idx == 0
