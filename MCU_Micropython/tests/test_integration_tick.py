import sys

if ".." not in sys.path:
    sys.path.append("..")

from util import assert_equal


def test_integration_tick():
    import gait_tripod

    gait = gait_tripod.TripodGait()
    gait.set_command(5.0, 0.0, 0.0)
    pose = gait.update(dt=0.02, send=False)
    assert_equal(len(pose), 6)
