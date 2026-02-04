import sys

# ============================
# USER CONFIG (edit as needed)
# ============================
EXPECTED_LEGS = 6
EXPECTED_SERVOS_PER_LEG = 3
EXPECTED_USED_SERVOS = 18
MIN_SERVO_ANGLE = 0.0
MAX_SERVO_ANGLE = 180.0
# ============================

if ".." not in sys.path:
    sys.path.append("..")

import config
from util import assert_equal, assert_true


def test_config_leg_mapping():
    assert_equal(len(config.LEG_SERVOS), EXPECTED_LEGS, "Expected 6 legs in LEG_SERVOS")
    flattened = []
    for idx, triplet in enumerate(config.LEG_SERVOS):
        assert_equal(
            len(triplet),
            EXPECTED_SERVOS_PER_LEG,
            "Leg %d should have 3 servo channels" % idx,
        )
        flattened.extend(list(triplet))
    assert_equal(
        len(flattened),
        EXPECTED_USED_SERVOS,
        "Expected %d total servo channels in LEG_SERVOS" % EXPECTED_USED_SERVOS,
    )
    assert_equal(
        len(set(flattened)),
        len(flattened),
        "Servo channel map contains duplicates",
    )


def test_config_servo_limits():
    assert_true(len(config.SERVO_MIN_A) == config.NUM_LOGICAL_CHANNELS)
    assert_true(len(config.SERVO_MAX_A) == config.NUM_LOGICAL_CHANNELS)
    for idx in range(config.NUM_LOGICAL_CHANNELS):
        lo = float(config.SERVO_MIN_A[idx])
        hi = float(config.SERVO_MAX_A[idx])
        assert_true(lo < hi, "SERVO_MIN_A[%d] must be < SERVO_MAX_A[%d]" % (idx, idx))
        assert_true(
            MIN_SERVO_ANGLE <= lo <= MAX_SERVO_ANGLE,
            "SERVO_MIN_A[%d] out of range" % idx,
        )
        assert_true(
            MIN_SERVO_ANGLE <= hi <= MAX_SERVO_ANGLE,
            "SERVO_MAX_A[%d] out of range" % idx,
        )


def test_config_geometry():
    assert_true(float(config.COXA_L) >= 0.0, "COXA_L should be >= 0")
    assert_true(float(config.FEMUR_L) > 0.0, "FEMUR_L must be > 0")
    assert_true(float(config.TIBIA_L) > 0.0, "TIBIA_L must be > 0")


def test_config_hip_positions():
    assert_equal(len(config.hip_pos_B), EXPECTED_LEGS, "Expected 6 hip positions")
    for idx, pos in enumerate(config.hip_pos_B):
        assert_equal(len(pos), 3, "hip_pos_B[%d] should be length 3" % idx)
