"""
Unit tests for simulation/backend.py

Run with:
    pytest tests/unit/test_simulation_backend.py -v
"""

import numpy as np
import pytest

from unified.simulation.backend import SimulationBackend, MockBackend


class TestMockBackendInit:
    """Tests for MockBackend initialization."""

    def test_default_num_envs(self):
        sim = MockBackend()
        assert sim.num_envs == 1

    def test_custom_num_envs(self):
        sim = MockBackend(num_envs=4)
        assert sim.num_envs == 4

    def test_initial_step_count_zero(self):
        sim = MockBackend(num_envs=2)
        assert sim.step_count == 0


class TestMockBackendReset:
    """Tests for MockBackend.reset()."""

    def test_reset_returns_obs_dict(self):
        sim = MockBackend(num_envs=2)
        obs = sim.reset()
        assert isinstance(obs, dict)
        assert "pos" in obs
        assert "quat" in obs
        assert "vel" in obs
        assert "ang_vel" in obs

    def test_reset_shapes(self):
        sim = MockBackend(num_envs=3)
        obs = sim.reset()
        assert obs["pos"].shape == (3, 3)
        assert obs["quat"].shape == (3, 4)
        assert obs["vel"].shape == (3, 3)
        assert obs["ang_vel"].shape == (3, 3)

    def test_reset_identity_quat(self):
        sim = MockBackend(num_envs=1)
        obs = sim.reset()
        # Quaternion [0,0,0,1] is identity
        np.testing.assert_array_almost_equal(obs["quat"], [[0.0, 0.0, 0.0, 1.0]])

    def test_reset_zeroes_positions(self):
        sim = MockBackend(num_envs=2)
        obs = sim.reset()
        np.testing.assert_array_almost_equal(obs["pos"], np.zeros((2, 3)))

    def test_reset_step_count_zero(self):
        sim = MockBackend(num_envs=1)
        sim._step_count = 99  # simulate prior steps
        sim.reset()
        assert sim.step_count == 0

    def test_reset_with_seed(self):
        sim = MockBackend(num_envs=1)
        obs1 = sim.reset(seed=42)
        obs2 = sim.reset(seed=42)
        np.testing.assert_array_almost_equal(obs1["pos"], obs2["pos"])


class TestMockBackendStep:
    """Tests for MockBackend.step()."""

    def test_step_returns_tuple(self):
        sim = MockBackend(num_envs=2)
        sim.reset()
        action = np.zeros((2, 4), dtype=np.float32)
        result = sim.step(action)
        assert isinstance(result, tuple)
        assert len(result) == 4

    def test_step_obs_shape(self):
        sim = MockBackend(num_envs=3)
        sim.reset()
        action = np.zeros((3, 4), dtype=np.float32)
        obs, reward, done, info = sim.step(action)
        assert obs["pos"].shape == (3, 3)

    def test_step_reward_done_shapes(self):
        sim = MockBackend(num_envs=4)
        sim.reset()
        action = np.zeros((4, 4), dtype=np.float32)
        obs, reward, done, info = sim.step(action)
        assert reward.shape == (4,)
        assert done.shape == (4,)

    def test_step_increments_count(self):
        sim = MockBackend(num_envs=1)
        sim.reset()
        assert sim.step_count == 0
        sim.step(np.zeros((1, 4), dtype=np.float32))
        assert sim.step_count == 1
        sim.step(np.zeros((1, 4), dtype=np.float32))
        assert sim.step_count == 2

    def test_step_thrust_increases_altitude(self):
        sim = MockBackend(num_envs=1, dt=0.1)
        sim.reset()
        action = np.array([[0.0, 0.0, 0.0, 10.0]], dtype=np.float32)  # thrust only
        obs1 = sim.get_obs()
        sim.step(action)
        obs2 = sim.get_obs()
        # Z position should increase
        assert obs2["pos"][0, 2] > obs1["pos"][0, 2]

    def test_step_wrong_shape_raises(self):
        sim = MockBackend(num_envs=2)
        sim.reset()
        bad_action = np.zeros((3, 4), dtype=np.float32)  # wrong num_envs
        with pytest.raises(ValueError):
            sim.step(bad_action)


class TestMockBackendGetObs:
    """Tests for MockBackend.get_obs()."""

    def test_get_obs_returns_dict(self):
        sim = MockBackend(num_envs=1)
        sim.reset()
        obs = sim.get_obs()
        assert isinstance(obs, dict)

    def test_get_obs_keys_match_crazyflow(self):
        sim = MockBackend(num_envs=2)
        sim.reset()
        obs = sim.get_obs()
        expected_keys = {"pos", "quat", "vel", "ang_vel"}
        assert set(obs.keys()) == expected_keys

    def test_get_obs_is_copy(self):
        sim = MockBackend(num_envs=1)
        sim.reset()
        obs = sim.get_obs()
        obs["pos"][0, 0] = 999.0
        obs2 = sim.get_obs()
        assert obs2["pos"][0, 0] != 999.0  # original unchanged


class TestMockBackendClose:
    """Tests for MockBackend.close()."""

    def test_close_does_not_raise(self):
        sim = MockBackend(num_envs=1)
        sim.reset()
        sim.close()  # should be no-op, no exception

    def test_close_is_idempotent(self):
        sim = MockBackend(num_envs=2)
        sim.reset()
        sim.close()
        sim.close()  # call twice


class TestSimulationBackendABC:
    """Verify SimulationBackend is abstract and MockBackend is concrete."""

    def test_cannot_instantiate_abc(self):
        with pytest.raises(TypeError):
            SimulationBackend(num_envs=1)

    def test_mock_is_subclass(self):
        assert issubclass(MockBackend, SimulationBackend)

    def test_mock_instance_is_sim_backend(self):
        sim = MockBackend(num_envs=1)
        assert isinstance(sim, SimulationBackend)
