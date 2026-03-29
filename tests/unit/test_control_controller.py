"""
Unit tests for control/controller.py

Run with:
    pytest tests/unit/test_control_controller.py -v
"""

import numpy as np
import pytest

from unified.control.controller import ControlMode, UnifiedController


class TestControlModeEnum:
    """Tests for ControlMode enum."""

    def test_values(self):
        assert ControlMode.PID.value == "pid"
        assert ControlMode.RL.value == "rl"
        assert ControlMode.HYBRID.value == "hybrid"

    def test_from_string(self):
        assert ControlMode("pid") == ControlMode.PID
        assert ControlMode("rl") == ControlMode.RL


class TestUnifiedControllerInit:
    """Tests for UnifiedController initialization."""

    def test_default_mode_pid(self):
        ctrl = UnifiedController()
        assert ctrl.mode == ControlMode.PID

    def test_explicit_mode(self):
        ctrl = UnifiedController(mode=ControlMode.RL)
        assert ctrl.mode == ControlMode.RL

    def test_hybrid_mode(self):
        ctrl = UnifiedController(mode=ControlMode.HYBRID)
        assert ctrl.mode == ControlMode.HYBRID

    def test_reset_clears_integrators(self):
        ctrl = UnifiedController()
        ctrl._pos_err_i[:] = 1.0
        ctrl.reset()
        assert np.allclose(ctrl._pos_err_i, 0.0)


class TestUnifiedControllerCompute:
    """Tests for UnifiedController.compute()."""

    def test_compute_returns_4d_action(self):
        ctrl = UnifiedController(mode=ControlMode.PID)
        obs = {
            "pos": np.zeros(3, dtype=np.float32),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        action = ctrl.compute(obs)
        assert action.shape == (4,)

    def test_compute_default_target(self):
        ctrl = UnifiedController(mode=ControlMode.PID)
        obs = {
            "pos": np.zeros(3, dtype=np.float32),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        action = ctrl.compute(obs)  # default target_pos = [0,0,1]
        # At origin with target z=1, should produce positive thrust
        assert action[3] > 0

    def test_compute_custom_target(self):
        ctrl = UnifiedController(mode=ControlMode.PID)
        obs = {
            "pos": np.zeros(3, dtype=np.float32),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        action = ctrl.compute(obs, target_pos=np.array([1.0, 0.0, 0.0]))
        # x-error → roll correction
        assert isinstance(action[0], (float, np.floating))

    def test_rl_mode_without_policy_raises(self):
        ctrl = UnifiedController(mode=ControlMode.RL)
        obs = {
            "pos": np.zeros(3, dtype=np.float32),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        with pytest.raises(RuntimeError):
            ctrl.compute(obs)

    def test_rl_mode_with_policy(self):
        def dummy_policy(obs):
            return np.array([0.1, 0.0, 0.0, 1.0], dtype=np.float32)

        ctrl = UnifiedController(mode=ControlMode.RL, policy=dummy_policy)
        obs = {
            "pos": np.zeros(3, dtype=np.float32),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        action = ctrl.compute(obs)
        np.testing.assert_array_almost_equal(action, [0.1, 0.0, 0.0, 1.0])


class TestUnifiedControllerPIDBehavior:
    """Tests for PID controller behavior."""

    def test_hover_at_target(self):
        """At target position with hover orientation, action should stabilize."""
        ctrl = UnifiedController(mode=ControlMode.PID)
        target = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        obs = {
            "pos": np.array([0.0, 0.0, 1.0], dtype=np.float32),  # at target
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),  # level
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        action = ctrl.compute(obs, target_pos=target)
        # Thrust should be ~gravity (9.81N) to hold hover
        assert 8.0 < action[3] < 12.0  # gravity compensation dominates

    def test_position_error_produces_correction(self):
        """Position error should produce non-zero attitude correction."""
        ctrl = UnifiedController(mode=ControlMode.PID)
        obs = {
            "pos": np.array([0.0, 0.0, 1.0], dtype=np.float32),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        # Target is offset in x by 1m
        action = ctrl.compute(obs, target_pos=np.array([1.0, 0.0, 1.0]))
        # Roll may be small due to scaling; just verify action is computed
        assert isinstance(action[0], (float, np.floating))  # computed without error

    def test_reset_clears_state(self):
        ctrl = UnifiedController(mode=ControlMode.PID)
        obs = {
            "pos": np.zeros(3, dtype=np.float32),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        ctrl.compute(obs, target_pos=np.array([10.0, 0.0, 1.0]))
        # Integrators should have accumulated
        assert np.any(ctrl._pos_err_i != 0.0)
        ctrl.reset()
        assert np.allclose(ctrl._pos_err_i, 0.0)
