import sys

if ".." not in sys.path:
    sys.path.append("..")

from util import assert_true


def test_ik_sanity():
    import ik

    sol = ik.ik_xyz(80.0, 0.0, -80.0)
    assert_true("coxa" in sol)
    assert_true("femur" in sol)
    assert_true("tibia" in sol)
