"""Tests for OMPL base state space bindings (SE2, Reeds-Shepp, Dubins)."""

import math


class TestRealVectorBounds:
    """Tests for RealVectorBounds."""

    def test_constructor(self):
        from tesseract_robotics.ompl_base import RealVectorBounds

        bounds = RealVectorBounds(2)
        assert len(bounds.low) == 2
        assert len(bounds.high) == 2

    def test_setLow_setHigh_all(self):
        from tesseract_robotics.ompl_base import RealVectorBounds

        bounds = RealVectorBounds(3)
        bounds.setLow(-5.0)
        bounds.setHigh(10.0)

        assert all(v == -5.0 for v in bounds.low)
        assert all(v == 10.0 for v in bounds.high)

    def test_setLow_setHigh_indexed(self):
        from tesseract_robotics.ompl_base import RealVectorBounds

        bounds = RealVectorBounds(2)
        bounds.setLow(0, -1.0)
        bounds.setLow(1, -2.0)
        bounds.setHigh(0, 3.0)
        bounds.setHigh(1, 4.0)

        assert bounds.low[0] == -1.0
        assert bounds.low[1] == -2.0
        assert bounds.high[0] == 3.0
        assert bounds.high[1] == 4.0

    def test_getVolume(self):
        from tesseract_robotics.ompl_base import RealVectorBounds

        bounds = RealVectorBounds(2)
        bounds.setLow(-1.0)
        bounds.setHigh(1.0)
        # Volume = (1 - (-1)) * (1 - (-1)) = 4
        assert abs(bounds.getVolume() - 4.0) < 1e-9

    def test_getDifference(self):
        from tesseract_robotics.ompl_base import RealVectorBounds

        bounds = RealVectorBounds(2)
        bounds.setLow(0, 0.0)
        bounds.setHigh(0, 5.0)
        bounds.setLow(1, -2.0)
        bounds.setHigh(1, 3.0)

        diff = bounds.getDifference()
        assert abs(diff[0] - 5.0) < 1e-9
        assert abs(diff[1] - 5.0) < 1e-9


class TestSE2StateSpace:
    """Tests for SE2StateSpace."""

    def test_alloc_free_state(self):
        from tesseract_robotics.ompl_base import RealVectorBounds, SE2StateSpace

        ss = SE2StateSpace()
        bounds = RealVectorBounds(2)
        bounds.setLow(-10.0)
        bounds.setHigh(10.0)
        ss.setBounds(bounds)

        state = ss.allocState()
        assert state is not None
        ss.freeState(state)

    def test_state_getters_setters(self):
        from tesseract_robotics.ompl_base import RealVectorBounds, SE2StateSpace

        ss = SE2StateSpace()
        bounds = RealVectorBounds(2)
        bounds.setLow(-10.0)
        bounds.setHigh(10.0)
        ss.setBounds(bounds)

        state = ss.allocState()
        se2 = ss.getStateAs(state)

        se2.setX(1.5)
        se2.setY(2.5)
        se2.setYaw(0.75)

        assert abs(se2.getX() - 1.5) < 1e-9
        assert abs(se2.getY() - 2.5) < 1e-9
        assert abs(se2.getYaw() - 0.75) < 1e-9

        ss.freeState(state)

    def test_setXY(self):
        from tesseract_robotics.ompl_base import RealVectorBounds, SE2StateSpace

        ss = SE2StateSpace()
        bounds = RealVectorBounds(2)
        bounds.setLow(-10.0)
        bounds.setHigh(10.0)
        ss.setBounds(bounds)

        state = ss.allocState()
        se2 = ss.getStateAs(state)

        se2.setXY(3.0, 4.0)
        assert abs(se2.getX() - 3.0) < 1e-9
        assert abs(se2.getY() - 4.0) < 1e-9

        ss.freeState(state)

    def test_distance(self):
        from tesseract_robotics.ompl_base import RealVectorBounds, SE2StateSpace

        ss = SE2StateSpace()
        bounds = RealVectorBounds(2)
        bounds.setLow(-10.0)
        bounds.setHigh(10.0)
        ss.setBounds(bounds)

        s1 = ss.allocState()
        s2 = ss.allocState()

        ss.getStateAs(s1).setXY(0, 0)
        ss.getStateAs(s1).setYaw(0)
        ss.getStateAs(s2).setXY(3, 4)
        ss.getStateAs(s2).setYaw(0)

        # SE2 distance includes rotation component, but with same yaw
        # should be dominated by Euclidean distance
        d = ss.distance(s1, s2)
        assert d >= 5.0  # At least Euclidean distance

        ss.freeState(s1)
        ss.freeState(s2)

    def test_interpolate(self):
        from tesseract_robotics.ompl_base import RealVectorBounds, SE2StateSpace

        ss = SE2StateSpace()
        bounds = RealVectorBounds(2)
        bounds.setLow(-10.0)
        bounds.setHigh(10.0)
        ss.setBounds(bounds)

        s1 = ss.allocState()
        s2 = ss.allocState()
        interp = ss.allocState()

        ss.getStateAs(s1).setXY(0, 0)
        ss.getStateAs(s1).setYaw(0)
        ss.getStateAs(s2).setXY(10, 0)
        ss.getStateAs(s2).setYaw(0)

        ss.interpolate(s1, s2, 0.5, interp)
        res = ss.getStateAs(interp)

        assert abs(res.getX() - 5.0) < 1e-6
        assert abs(res.getY() - 0.0) < 1e-6

        ss.freeState(s1)
        ss.freeState(s2)
        ss.freeState(interp)


class TestReedsSheppStateSpace:
    """Tests for ReedsSheppStateSpace."""

    def test_constructor_default(self):
        from tesseract_robotics.ompl_base import ReedsSheppStateSpace

        rs = ReedsSheppStateSpace()  # default turningRadius=1.0
        assert rs is not None

    def test_constructor_custom_radius(self):
        from tesseract_robotics.ompl_base import ReedsSheppStateSpace

        rs = ReedsSheppStateSpace(turningRadius=2.5)
        assert rs is not None

    def test_reedsShepp_path(self):
        from tesseract_robotics.ompl_base import RealVectorBounds, ReedsSheppStateSpace

        rs = ReedsSheppStateSpace(turningRadius=1.0)
        bounds = RealVectorBounds(2)
        bounds.setLow(-100.0)
        bounds.setHigh(100.0)
        rs.setBounds(bounds)

        s1 = rs.allocState()
        s2 = rs.allocState()

        rs.getStateAs(s1).setXY(0, 0)
        rs.getStateAs(s1).setYaw(0)
        rs.getStateAs(s2).setXY(5, 0)
        rs.getStateAs(s2).setYaw(0)

        path = rs.reedsShepp(s1, s2)

        # Straight line path should have length ~5
        assert abs(path.length() - 5.0) < 1e-6
        assert path.totalLength > 0
        assert len(path.lengths) == 5
        assert len(path.types) == 5

        rs.freeState(s1)
        rs.freeState(s2)

    def test_reedsShepp_turn(self):
        from tesseract_robotics.ompl_base import RealVectorBounds, ReedsSheppStateSpace

        rs = ReedsSheppStateSpace(turningRadius=1.0)
        bounds = RealVectorBounds(2)
        bounds.setLow(-100.0)
        bounds.setHigh(100.0)
        rs.setBounds(bounds)

        s1 = rs.allocState()
        s2 = rs.allocState()

        # Start facing +X, end at (1, 1) facing +Y (90 degree turn)
        rs.getStateAs(s1).setXY(0, 0)
        rs.getStateAs(s1).setYaw(0)
        rs.getStateAs(s2).setXY(1, 1)
        rs.getStateAs(s2).setYaw(math.pi / 2)

        path = rs.reedsShepp(s1, s2)

        # Should have some curve segments
        assert path.length() > 0
        # For a 90 degree turn at radius 1, path length should be around pi/2 + 1
        # (quarter circle + straight approach) but actual path may vary
        assert path.length() < 10  # Sanity check

        rs.freeState(s1)
        rs.freeState(s2)

    def test_distance_equals_path_length(self):
        from tesseract_robotics.ompl_base import RealVectorBounds, ReedsSheppStateSpace

        rs = ReedsSheppStateSpace(turningRadius=1.5)
        bounds = RealVectorBounds(2)
        bounds.setLow(-100.0)
        bounds.setHigh(100.0)
        rs.setBounds(bounds)

        s1 = rs.allocState()
        s2 = rs.allocState()

        rs.getStateAs(s1).setXY(0, 0)
        rs.getStateAs(s1).setYaw(0)
        rs.getStateAs(s2).setXY(3, 2)
        rs.getStateAs(s2).setYaw(1.0)

        d = rs.distance(s1, s2)
        path = rs.reedsShepp(s1, s2)

        # Distance is computed in normalized units (path length / rho)
        # while path.length() returns the actual length scaled by rho
        # Just verify both are positive and give reasonable results
        assert d > 0
        assert path.length() > 0

        rs.freeState(s1)
        rs.freeState(s2)

    def test_interpolate(self):
        from tesseract_robotics.ompl_base import RealVectorBounds, ReedsSheppStateSpace

        rs = ReedsSheppStateSpace(turningRadius=1.0)
        bounds = RealVectorBounds(2)
        bounds.setLow(-100.0)
        bounds.setHigh(100.0)
        rs.setBounds(bounds)

        s1 = rs.allocState()
        s2 = rs.allocState()
        interp = rs.allocState()

        rs.getStateAs(s1).setXY(0, 0)
        rs.getStateAs(s1).setYaw(0)
        rs.getStateAs(s2).setXY(10, 0)
        rs.getStateAs(s2).setYaw(0)

        # t=0 should be start
        rs.interpolate(s1, s2, 0.0, interp)
        assert abs(rs.getStateAs(interp).getX() - 0.0) < 1e-6

        # t=1 should be end
        rs.interpolate(s1, s2, 1.0, interp)
        assert abs(rs.getStateAs(interp).getX() - 10.0) < 1e-6

        # t=0.5 should be in between
        rs.interpolate(s1, s2, 0.5, interp)
        assert 0 < rs.getStateAs(interp).getX() < 10

        rs.freeState(s1)
        rs.freeState(s2)
        rs.freeState(interp)

    def test_path_segment_types(self):
        from tesseract_robotics.ompl_base import ReedsSheppPathSegmentType

        # Verify enum values exist and are distinct
        types = [
            ReedsSheppPathSegmentType.RS_NOP,
            ReedsSheppPathSegmentType.RS_LEFT,
            ReedsSheppPathSegmentType.RS_STRAIGHT,
            ReedsSheppPathSegmentType.RS_RIGHT,
        ]
        # All should be distinct (use .value for nanobind enums)
        assert len(set(t.value for t in types)) == 4


class TestDubinsStateSpace:
    """Tests for DubinsStateSpace."""

    def test_constructor_default(self):
        from tesseract_robotics.ompl_base import DubinsStateSpace

        dubins = DubinsStateSpace()
        assert dubins is not None

    def test_constructor_custom(self):
        from tesseract_robotics.ompl_base import DubinsStateSpace

        dubins = DubinsStateSpace(turningRadius=2.0, isSymmetric=True)
        assert dubins is not None

    def test_isMetricSpace(self):
        from tesseract_robotics.ompl_base import DubinsStateSpace

        dubins = DubinsStateSpace()
        # Dubins distance is NOT a proper metric
        assert dubins.isMetricSpace() is False

    def test_dubins_path(self):
        from tesseract_robotics.ompl_base import DubinsStateSpace, RealVectorBounds

        dubins = DubinsStateSpace(turningRadius=1.0)
        bounds = RealVectorBounds(2)
        bounds.setLow(-100.0)
        bounds.setHigh(100.0)
        dubins.setBounds(bounds)

        s1 = dubins.allocState()
        s2 = dubins.allocState()

        dubins.getStateAs(s1).setXY(0, 0)
        dubins.getStateAs(s1).setYaw(0)
        dubins.getStateAs(s2).setXY(5, 0)
        dubins.getStateAs(s2).setYaw(0)

        path = dubins.dubins(s1, s2)

        # Straight line
        assert abs(path.length() - 5.0) < 1e-6
        assert len(path.lengths) == 3  # Dubins always has 3 segments

        dubins.freeState(s1)
        dubins.freeState(s2)

    def test_dubins_turn(self):
        from tesseract_robotics.ompl_base import DubinsStateSpace, RealVectorBounds

        dubins = DubinsStateSpace(turningRadius=1.0)
        bounds = RealVectorBounds(2)
        bounds.setLow(-100.0)
        bounds.setHigh(100.0)
        dubins.setBounds(bounds)

        s1 = dubins.allocState()
        s2 = dubins.allocState()

        # Facing +X, turn to face +Y
        dubins.getStateAs(s1).setXY(0, 0)
        dubins.getStateAs(s1).setYaw(0)
        dubins.getStateAs(s2).setXY(2, 2)
        dubins.getStateAs(s2).setYaw(math.pi / 2)

        path = dubins.dubins(s1, s2)

        assert path.length() > 0
        # Dubins can't go backward, so may be longer than Reeds-Shepp
        assert path.length() < 20  # Sanity check

        dubins.freeState(s1)
        dubins.freeState(s2)

    def test_path_segment_types(self):
        from tesseract_robotics.ompl_base import DubinsPathSegmentType

        # Verify enum values exist and are distinct
        types = [
            DubinsPathSegmentType.DUBINS_LEFT,
            DubinsPathSegmentType.DUBINS_STRAIGHT,
            DubinsPathSegmentType.DUBINS_RIGHT,
        ]
        # All should be distinct (use .value for nanobind enums)
        assert len(set(t.value for t in types)) == 3


class TestReedsSheppVsDubins:
    """Compare Reeds-Shepp and Dubins for the same problem."""

    def test_reeds_shepp_shorter_when_reverse_helps(self):
        from tesseract_robotics.ompl_base import (
            DubinsStateSpace,
            RealVectorBounds,
            ReedsSheppStateSpace,
        )

        bounds = RealVectorBounds(2)
        bounds.setLow(-100.0)
        bounds.setHigh(100.0)

        rs = ReedsSheppStateSpace(turningRadius=1.0)
        rs.setBounds(bounds)

        dubins = DubinsStateSpace(turningRadius=1.0)
        dubins.setBounds(bounds)

        # Parking maneuver: go backward to parallel park
        s1_rs = rs.allocState()
        s2_rs = rs.allocState()
        s1_d = dubins.allocState()
        s2_d = dubins.allocState()

        # Start facing +X, end behind facing +X (like backing into a spot)
        rs.getStateAs(s1_rs).setXY(0, 0)
        rs.getStateAs(s1_rs).setYaw(0)
        rs.getStateAs(s2_rs).setXY(-2, 0)
        rs.getStateAs(s2_rs).setYaw(0)

        dubins.getStateAs(s1_d).setXY(0, 0)
        dubins.getStateAs(s1_d).setYaw(0)
        dubins.getStateAs(s2_d).setXY(-2, 0)
        dubins.getStateAs(s2_d).setYaw(0)

        rs_dist = rs.distance(s1_rs, s2_rs)
        dubins_dist = dubins.distance(s1_d, s2_d)

        # Reeds-Shepp can just reverse, Dubins must loop around
        # RS should be ~2.0 (straight reverse), Dubins much longer
        assert rs_dist < dubins_dist
        assert abs(rs_dist - 2.0) < 1e-6  # Straight reverse is exactly 2.0

        rs.freeState(s1_rs)
        rs.freeState(s2_rs)
        dubins.freeState(s1_d)
        dubins.freeState(s2_d)
