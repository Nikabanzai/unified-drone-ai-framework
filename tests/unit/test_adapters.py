"""
Unit tests for state_adapter.py and action_adapter.py.

Run with:
    pytest test_adapters.py -v

All tests use only numpy + pytest (no simulation imports).
"""

import numpy as np
import pytest

# Import adapters from unified package (pure numpy, no sim)
from unified.adapters.state_adapter import (
    gympybullet_to_crazyflow,
    crazyflow_to_gympybullet,
    _quat_to_rpy,
)
from unified.adapters.action_adapter import (
    rpm_to_attitude,
    attitude_to_rpm,
    get_mixer_matrix,
    CF2X_MIXER,
    DEFAULT_HOVER_RPM,
)


# =============================================================================
# State Adapter Tests
# =============================================================================

class TestStateAdapterGymPyBulletToCrazyflow:
    """Tests for gympybullet_to_crazyflow()."""

    def test_single_drone_shape(self):
        """Test that (20,) obs converts to dict with correct shapes."""
        # Create a sample observation matching CtrlAviary._getDroneStateVector format
        obs = np.zeros(20, dtype=np.float32)
        obs[0:3] = [1.0, 2.0, 3.0]  # pos
        obs[3:7] = [0.0, 0.0, 0.0, 1.0]  # quat (identity)
        obs[7:10] = [0.0, 0.0, 0.0]  # rpy (not used by crazyflow)
        obs[10:13] = [0.5, 0.0, 0.0]  # vel
        obs[13:16] = [0.0, 0.1, 0.0]  # ang_vel
        obs[16:20] = [14000.0, 14468.0, 14468.0, 14468.0]  # rpm (not used by crazyflow)

        result = gympybullet_to_crazyflow(obs)

        assert isinstance(result, dict)
        assert result["pos"].shape == (3,)
        assert result["quat"].shape == (4,)
        assert result["vel"].shape == (3,)
        assert result["ang_vel"].shape == (3,)

    def test_single_drone_values_preserved(self):
        """Test that values are correctly extracted from (20,) obs."""
        obs = np.zeros(20, dtype=np.float32)
        obs[0:3] = [5.0, -3.0, 2.5]  # pos
        obs[3:7] = [0.1, 0.2, 0.3, 0.9]  # quat (normalized-ish)
        obs[10:13] = [1.0, 2.0, 3.0]  # vel
        obs[13:16] = [0.5, 0.6, 0.7]  # ang_vel

        result = gympybullet_to_crazyflow(obs)

        np.testing.assert_array_almost_equal(result["pos"], [5.0, -3.0, 2.5])
        np.testing.assert_array_almost_equal(result["quat"], [0.1, 0.2, 0.3, 0.9])
        np.testing.assert_array_almost_equal(result["vel"], [1.0, 2.0, 3.0])
        np.testing.assert_array_almost_equal(result["ang_vel"], [0.5, 0.6, 0.7])

    def test_batched_shape(self):
        """Test that (N, 20) obs converts to dict of (N, ...) arrays."""
        n_drones = 3
        obs = np.zeros((n_drones, 20), dtype=np.float32)
        obs[:, 0:3] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # different pos per drone
        obs[:, 3:7] = [[0, 0, 0, 1]] * n_drones  # same quat
        obs[:, 10:13] = [[0.1, 0.2, 0.3]] * n_drones
        obs[:, 13:16] = [[0.01, 0.02, 0.03]] * n_drones

        result = gympybullet_to_crazyflow(obs)

        assert result["pos"].shape == (n_drones, 3)
        assert result["quat"].shape == (n_drones, 4)
        assert result["vel"].shape == (n_drones, 3)
        assert result["ang_vel"].shape == (n_drones, 3)

        np.testing.assert_array_almost_equal(result["pos"], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def test_invalid_shape_raises(self):
        """Test that wrong shape raises ValueError."""
        bad_obs = np.zeros(19, dtype=np.float32)  # too short
        with pytest.raises(ValueError):
            gympybullet_to_crazyflow(bad_obs)


class TestStateAdapterCrazyflowToGymPyBullet:
    """Tests for crazyflow_to_gympybullet()."""

    def test_single_drone_shape(self):
        """Test that dict obs converts to (20,) array."""
        obs = {
            "pos": np.array([1.0, 2.0, 3.0], dtype=np.float32),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "vel": np.array([0.0, 0.0, 0.0], dtype=np.float32),
            "ang_vel": np.array([0.0, 0.0, 0.0], dtype=np.float32),
        }

        result = crazyflow_to_gympybullet(obs)

        assert result.shape == (20,)
        assert result.dtype == np.float32

    def test_single_drone_values_preserved(self):
        """Test that values are correctly placed in (20,) array."""
        obs = {
            "pos": np.array([5.0, -3.0, 2.5], dtype=np.float32),
            "quat": np.array([0.1, 0.2, 0.3, 0.9], dtype=np.float32),
            "vel": np.array([1.0, 2.0, 3.0], dtype=np.float32),
            "ang_vel": np.array([0.5, 0.6, 0.7], dtype=np.float32),
        }

        result = crazyflow_to_gympybullet(obs)

        np.testing.assert_array_almost_equal(result[0:3], [5.0, -3.0, 2.5])  # pos
        np.testing.assert_array_almost_equal(result[3:7], [0.1, 0.2, 0.3, 0.9])  # quat
        np.testing.assert_array_almost_equal(result[10:13], [1.0, 2.0, 3.0])  # vel
        np.testing.assert_array_almost_equal(result[13:16], [0.5, 0.6, 0.7])  # ang_vel
        # RPY (7-9) and RPM (16-19) are computed/filled

    def test_rpm_optional(self):
        """Test that rpm can be optionally provided."""
        obs = {
            "pos": np.zeros(3, dtype=np.float32),
            "quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        rpm = np.array([14000.0, 14468.0, 14468.0, 14468.0], dtype=np.float32)

        result = crazyflow_to_gympybullet(obs, rpm=rpm)

        np.testing.assert_array_almost_equal(result[16:20], rpm)

    def test_batched_shape(self):
        """Test that batched dict converts to (N, 20) array."""
        n_drones = 2
        obs = {
            "pos": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32),
            "quat": np.array([[0.0, 0.0, 0.0, 1.0]] * n_drones, dtype=np.float32),
            "vel": np.zeros((n_drones, 3), dtype=np.float32),
            "ang_vel": np.zeros((n_drones, 3), dtype=np.float32),
        }

        result = crazyflow_to_gympybullet(obs)

        assert result.shape == (n_drones, 20)

    def test_missing_key_raises(self):
        """Test that missing key raises KeyError."""
        obs = {
            "pos": np.zeros(3, dtype=np.float32),
            # Missing "quat"
            "vel": np.zeros(3, dtype=np.float32),
            "ang_vel": np.zeros(3, dtype=np.float32),
        }
        with pytest.raises(KeyError):
            crazyflow_to_gympybullet(obs)


class TestQuatToRpy:
    """Tests for _quat_to_rpy() helper."""

    def test_identity_quat(self):
        """Identity quaternion [0,0,0,1] → [0,0,0] RPY."""
        quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        rpy = _quat_to_rpy(quat)
        np.testing.assert_array_almost_equal(rpy, [0.0, 0.0, 0.0], decimal=5)

    def test_roll_90_deg(self):
        """Quaternion for 90° roll → [pi/2, 0, 0]."""
        # Rotation around X by 90°: [sin(45°), 0, 0, cos(45°)]
        quat = np.array([np.sin(np.pi / 4), 0.0, 0.0, np.cos(np.pi / 4)], dtype=np.float32)
        rpy = _quat_to_rpy(quat)
        np.testing.assert_array_almost_equal(rpy, [np.pi / 2, 0.0, 0.0], decimal=4)

    def test_yaw_45_deg(self):
        """Quaternion for 45° yaw → [0, 0, pi/4]."""
        quat = np.array([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)], dtype=np.float32)
        rpy = _quat_to_rpy(quat)
        np.testing.assert_array_almost_equal(rpy, [0.0, 0.0, np.pi / 4], decimal=4)


# =============================================================================
# Action Adapter Tests
# =============================================================================

class TestActionAdapterRpmToAttitude:
    """Tests for rpm_to_attitude()."""

    def test_single_drone_shape(self):
        """Test (4,) RPMs → (4,) attitude."""
        rpms = np.array([14468.0, 14468.0, 14468.0, 14468.0], dtype=np.float32)  # hover
        result = rpm_to_attitude(rpms)

        assert result.shape == (4,)
        assert result.dtype == np.float32
        # At hover, roll/pitch/yaw should be ~0
        np.testing.assert_array_almost_equal(result[:3], [0.0, 0.0, 0.0], decimal=3)

    def test_hover_rpm_gives_zero_attitude(self):
        """Hover RPMs (all equal) → near-zero roll/pitch/yaw."""
        hover_rpms = np.full(4, DEFAULT_HOVER_RPM, dtype=np.float32)
        result = rpm_to_attitude(hover_rpms)

        # Roll, pitch, yaw should be approximately zero
        assert abs(result[0]) < 1e-6  # roll
        assert abs(result[1]) < 1e-6  # pitch
        assert abs(result[2]) < 1e-6  # yaw
        # Thrust should be ~0.5N at hover
        assert abs(result[3] - 0.5) < 0.1

    def test_batched_shape(self):
        """Test (N, 4) RPMs → (N, 4) attitude."""
        n = 3
        rpms = np.full((n, 4), DEFAULT_HOVER_RPM, dtype=np.float32)
        result = rpm_to_attitude(rpms)

        assert result.shape == (n, 4)

    def test_cf2p_model_supported(self):
        """Test that CF2P drone model works."""
        rpms = np.full(4, DEFAULT_HOVER_RPM, dtype=np.float32)
        result = rpm_to_attitude(rpms, drone_model="CF2P")
        assert result.shape == (4,)

    def test_invalid_model_raises(self):
        """Test unknown drone model raises ValueError."""
        rpms = np.zeros(4, dtype=np.float32)
        with pytest.raises(ValueError):
            rpm_to_attitude(rpms, drone_model="UNKNOWN")


class TestActionAdapterAttitudeToRpm:
    """Tests for attitude_to_rpm()."""

    def test_single_drone_shape(self):
        """Test (4,) attitude → (4,) RPMs."""
        attitude = np.array([0.0, 0.0, 0.0, 0.5], dtype=np.float32)  # hover
        result = attitude_to_rpm(attitude)

        assert result.shape == (4,)
        assert result.dtype == np.float32

    def test_zero_attitude_gives_hover_rpms(self):
        """Zero attitude [0,0,0,0.5] → all RPMs ≈ hover."""
        attitude = np.array([0.0, 0.0, 0.0, 0.5], dtype=np.float32)
        result = attitude_to_rpm(attitude, hover_rpm=DEFAULT_HOVER_RPM)

        # All RPMs should be close to hover RPM
        np.testing.assert_array_almost_equal(result, [DEFAULT_HOVER_RPM] * 4, decimal=-1)

    def test_roll_produces_differential_rpms(self):
        """Non-zero roll produces differential RPMs (P0≠P2, P1≠P3)."""
        roll = 0.1  # radians
        attitude = np.array([roll, 0.0, 0.0, 0.5], dtype=np.float32)
        result = attitude_to_rpm(attitude)

        # For CF2X mixer, roll positive: P0/P1 decrease, P2/P3 increase
        # P0 and P1 should be less than P2 and P3
        assert result[0] < result[2]  # P0 < P2
        assert result[1] < result[3]  # P1 < P3

    def test_batched_shape(self):
        """Test (N, 4) attitude → (N, 4) RPMs."""
        n = 2
        attitude = np.zeros((n, 4), dtype=np.float32)
        attitude[:, 3] = 0.5  # thrust
        result = attitude_to_rpm(attitude)

        assert result.shape == (n, 4)

    def test_non_negative_rpms(self):
        """RPMs should never be negative (clipped)."""
        # Large negative roll could underflow
        attitude = np.array([-10.0, 0.0, 0.0, 0.1], dtype=np.float32)
        result = attitude_to_rpm(attitude)

        assert np.all(result >= 0.0)


class TestActionAdapterMixerMatrix:
    """Tests for get_mixer_matrix()."""

    def test_cf2x_mixer_shape(self):
        """CF2X mixer is (4, 3)."""
        mixer = get_mixer_matrix("CF2X")
        assert mixer.shape == (4, 3)

    def test_cf2x_mixer_values_match_dslpid(self):
        """Mixer values match DSLPIDControl for CF2X."""
        mixer = get_mixer_matrix("CF2X")
        np.testing.assert_array_almost_equal(mixer, CF2X_MIXER)

    def test_cf2p_mixer_shape(self):
        """CF2P mixer is (4, 3)."""
        mixer = get_mixer_matrix("CF2P")
        assert mixer.shape == (4, 3)

    def test_invalid_model_raises(self):
        """Unknown model raises ValueError."""
        with pytest.raises(ValueError):
            get_mixer_matrix("CF2Z")


class TestActionAdapterRoundTrip:
    """Tests for RPM ↔ Attitude round-trip consistency."""

    def test_rpm_to_attitude_to_rpm_near_hover(self):
        """RPM → Attitude → RPM should approximately recover original (near hover)."""
        original_rpms = np.array([14400.0, 14500.0, 14450.0, 14520.0], dtype=np.float32)
        attitude = rpm_to_attitude(original_rpms)
        recovered_rpms = attitude_to_rpm(attitude)

        # Near hover, the round-trip should be close (not exact due to approximations)
        np.testing.assert_array_almost_equal(recovered_rpms, original_rpms, decimal=-2)

    def test_attitude_to_rpm_to_attitude_zero(self):
        """Zero attitude → RPM → Attitude should recover zeros."""
        original = np.array([0.0, 0.0, 0.0, 0.5], dtype=np.float32)
        rpms = attitude_to_rpm(original)
        recovered = rpm_to_attitude(rpms)

        np.testing.assert_array_almost_equal(recovered[:3], original[:3], decimal=4)


# =============================================================================
# Integration / Shape Compatibility Tests
# =============================================================================

class TestAdapterIntegration:
    """Tests that adapters produce shapes compatible with repo interfaces."""

    def test_gympybullet_obs_shape_matches_repo(self):
        """(20,) is the documented obs shape from CtrlAviary._getDroneStateVector."""
        # Simulate what CtrlAviary would produce
        obs = np.random.randn(20).astype(np.float32)
        result = gympybullet_to_crazyflow(obs)
        # Should be convertible back
        back = crazyflow_to_gympybullet(result)
        assert back.shape == (20,)

    def test_crazyflow_obs_shape_matches_repo(self):
        """Dict with pos/quat/vel/ang_vel matches DroneEnv.single_observation_space."""
        cf_obs = {
            "pos": np.random.randn(3).astype(np.float32),
            "quat": np.random.randn(4).astype(np.float32),
            "vel": np.random.randn(3).astype(np.float32),
            "ang_vel": np.random.randn(3).astype(np.float32),
        }
        result = crazyflow_to_gympybullet(cf_obs)
        assert result.shape == (20,)

    def test_rpm_action_shape_matches_repo(self):
        """(N, 4) RPMs match CtrlAviary._actionSpace."""
        rpms = np.random.rand(2, 4).astype(np.float32) * 20000  # typical RPM range
        attitude = rpm_to_attitude(rpms)
        assert attitude.shape == (2, 4)

    def test_attitude_action_shape_matches_repo(self):
        """(N, 4) attitude matches crazyflow DroneEnv attitude action space."""
        attitude = np.array([[0.1, 0.0, 0.0, 0.5], [0.0, -0.1, 0.0, 0.5]], dtype=np.float32)
        rpms = attitude_to_rpm(attitude)
        assert rpms.shape == (2, 4)
