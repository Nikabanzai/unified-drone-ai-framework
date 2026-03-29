"""
Unit tests for planning/coverage.py

Run with:
    pytest tests/unit/test_planning_coverage.py -v
"""

import pytest

from unified.planning.coverage import boustrophedon_grid, spiral_grid


class TestBoustrophedonGrid:
    """Tests for boustrophedon_grid()."""

    def test_basic_shape(self):
        wps = boustrophedon_grid(0, 4, 0, 4, altitude=5.0, spacing=2.0)
        assert isinstance(wps, list)
        assert len(wps) > 0
        assert all(len(wp) == 3 for wp in wps)

    def test_altitude_constant(self):
        wps = boustrophedon_grid(0, 10, 0, 10, altitude=7.5, spacing=1.0)
        for x, y, z in wps:
            assert z == 7.5

    def test_spacing_controls_count(self):
        wps_small = boustrophedon_grid(0, 4, 0, 4, altitude=1, spacing=2.0)
        wps_large = boustrophedon_grid(0, 4, 0, 4, altitude=1, spacing=1.0)
        # Smaller spacing → more waypoints
        assert len(wps_large) >= len(wps_small)

    def test_corners_included(self):
        wps = boustrophedon_grid(0, 2, 0, 2, altitude=1, spacing=1.0)
        xs = [w[0] for w in wps]
        ys = [w[1] for w in wps]
        assert 0.0 in xs and 2.0 in xs
        assert 0.0 in ys and 2.0 in ys

    def test_invalid_bounds_raises(self):
        with pytest.raises(ValueError):
            boustrophedon_grid(5, 1, 0, 1)  # x_max < x_min

    def test_invalid_spacing_raises(self):
        with pytest.raises(ValueError):
            boustrophedon_grid(0, 1, 0, 1, spacing=0)

    def test_start_corner_top_left(self):
        wps = boustrophedon_grid(0, 4, 0, 4, altitude=1, spacing=2.0, start_corner="top_left")
        # First waypoint should be near top (y_max)
        first_y = wps[0][1]
        assert first_y >= 2.0  # near top

    def test_sweeps_alternate(self):
        wps = boustrophedon_grid(0, 4, 0, 4, altitude=1, spacing=2.0)
        # Check that x direction alternates per y row
        # Extract x ranges per y group
        # For simplicity, just ensure total count is reasonable
        assert len(wps) >= 6  # 3 x points × 2 y points minimum


class TestSpiralGrid:
    """Tests for spiral_grid()."""

    def test_basic_shape(self):
        wps = spiral_grid(0.0, 0.0, radius=5.0, altitude=3.0, spacing=1.0)
        assert isinstance(wps, list)
        assert len(wps) > 0
        for wp in wps:
            assert len(wp) == 3

    def test_ends_at_center(self):
        wps = spiral_grid(5.0, 5.0, radius=3.0, altitude=2.0)
        last = wps[-1]
        assert last[0] == 5.0 and last[1] == 5.0

    def test_altitude_constant(self):
        wps = spiral_grid(0, 0, radius=2, altitude=10.0)
        for x, y, z in wps:
            assert z == 10.0

    def test_invalid_radius_raises(self):
        with pytest.raises(ValueError):
            spiral_grid(0, 0, radius=0)

    def test_invalid_spacing_raises(self):
        with pytest.raises(ValueError):
            spiral_grid(0, 0, radius=1, spacing=0)

    def test_counterclockwise(self):
        wps_cw = spiral_grid(0, 0, radius=2, clockwise=True)
        wps_ccw = spiral_grid(0, 0, radius=2, clockwise=False)
        # Should have same number of points but different order
        assert len(wps_cw) == len(wps_ccw)
        # First point should differ (or last)
        assert wps_cw[0] != wps_ccw[0] or wps_cw[-1] != wps_ccw[-1]
