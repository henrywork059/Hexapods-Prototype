import sys

# ============================
# USER CONFIG (edit as needed)
# ============================
START = (0.0, 0.0, -50.0)
END = (30.0, 0.0, -50.0)
CLEARANCE = 12.0
SAMPLES = 11
TOL = 1e-3
# ============================

if ".." not in sys.path:
    sys.path.append("..")

import trajectory
from util import assert_almost_equal, assert_true


def test_swing_path_endpoints_and_clearance():
    p0 = trajectory.swing(START, END, 0.0, CLEARANCE, style="sine", ease_kind="quintic")
    p1 = trajectory.swing(START, END, 1.0, CLEARANCE, style="sine", ease_kind="quintic")
    mid = trajectory.swing(START, END, 0.5, CLEARANCE, style="sine", ease_kind="quintic")

    assert_almost_equal(p0[0], START[0], tol=TOL)
    assert_almost_equal(p0[1], START[1], tol=TOL)
    assert_almost_equal(p0[2], START[2], tol=TOL)

    assert_almost_equal(p1[0], END[0], tol=TOL)
    assert_almost_equal(p1[1], END[1], tol=TOL)
    assert_almost_equal(p1[2], END[2], tol=TOL)

    assert_almost_equal(mid[2], START[2] + CLEARANCE, tol=TOL)


def test_swing_xy_monotonic():
    xs = []
    for k in range(SAMPLES):
        t = k / float(SAMPLES - 1)
        p = trajectory.swing(START, END, t, CLEARANCE, style="sine", ease_kind="quintic")
        xs.append(p[0])

    for i in range(1, len(xs)):
        assert_true(xs[i] >= xs[i - 1] - TOL, "X should be monotonic along swing")
