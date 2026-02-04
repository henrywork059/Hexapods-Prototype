import sys

if ".." not in sys.path:
    sys.path.append("..")

from util import assert_almost_equal


def test_swing_midpoint_lift():
    import trajectory

    p0 = (0.0, 0.0, -50.0)
    p1 = (20.0, 0.0, -50.0)
    h = 10.0
    mid = trajectory.swing_parabolic(p0, p1, 0.5, h)
    assert_almost_equal(mid[2], -40.0, tol=1e-3)
