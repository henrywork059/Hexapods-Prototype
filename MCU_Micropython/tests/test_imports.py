import sys

if ".." not in sys.path:
    sys.path.append("..")

from util import assert_true


def test_imports():
    import config
    import frames
    import gait_tripod
    import ik
    import servo_pca
    import trajectory

    assert_true(hasattr(config, "LEG_SERVOS"))
    assert_true(hasattr(gait_tripod, "TripodGait"))
    assert_true(hasattr(ik, "ik_xyz"))
    assert_true(hasattr(servo_pca, "set_servo"))
    assert_true(hasattr(trajectory, "swing"))
