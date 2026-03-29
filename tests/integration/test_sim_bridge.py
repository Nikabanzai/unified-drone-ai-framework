"""
Integration tests: import sanity (no simulation runs).

These tests verify that all modules can be imported without heavy dependencies
(Isaac Sim, ROS, PX4). They should pass even if crazyflow/gym-pybullet-drones
are not installed.
"""

import pytest


class TestImportSanity:
    """Verify all unified modules import cleanly."""

    def test_adapters_import(self):
        from unified.adapters.state_adapter import gympybullet_to_crazyflow
        from unified.adapters.action_adapter import rpm_to_attitude
        assert callable(gympybullet_to_crazyflow)
        assert callable(rpm_to_attitude)

    def test_simulation_enum_import(self):
        from unified.simulation import Backend
        assert Backend.CRAZYFLOW.name == "CRAZYFLOW"
        assert Backend.PYBULLET.name == "PYBULLET"

    def test_control_enum_import(self):
        from unified.control import ControlMode
        assert ControlMode.PID.name == "PID"
        assert ControlMode.RL.name == "RL"
        assert ControlMode.HYBRID.name == "HYBRID"

    def test_planning_stub_import(self):
        from unified.planning import __all__ as planning_all
        assert isinstance(planning_all, list)

    def test_mission_stub_import(self):
        from unified.mission import __all__ as mission_all
        assert isinstance(mission_all, list)

    def test_top_level_import(self):
        import unified
        assert hasattr(unified, "__version__")
        assert unified.__version__ == "0.1.0"
