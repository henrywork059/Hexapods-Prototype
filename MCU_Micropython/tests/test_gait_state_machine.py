import sys

if ".." not in sys.path:
    sys.path.append("..")

from util import assert_equal, assert_true


def test_gait_basic_update():
    import gait_tripod

    gait = gait_tripod.TripodGait()
    gait.set_command(10.0, 0.0, 0.0)
    pose = gait.update(dt=0.02, send=False)
    assert_equal(len(pose), 6)
    assert_true(all(len(p) == 3 for p in pose))
