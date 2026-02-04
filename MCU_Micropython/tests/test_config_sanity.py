import sys

if ".." not in sys.path:
    sys.path.append("..")

import config
from util import assert_equal, assert_true


def test_config_leg_mapping():
    assert_equal(len(config.LEG_SERVOS), 6, "Expected 6 legs in LEG_SERVOS")
    for idx, triplet in enumerate(config.LEG_SERVOS):
        assert_equal(len(triplet), 3, "Leg %d should have 3 servo channels" % idx)


def test_config_servo_limits():
    assert_true(len(config.SERVO_MIN_A) == config.NUM_LOGICAL_CHANNELS)
    assert_true(len(config.SERVO_MAX_A) == config.NUM_LOGICAL_CHANNELS)


def test_config_hip_positions():
    assert_equal(len(config.hip_pos_B), 6, "Expected 6 hip positions")
