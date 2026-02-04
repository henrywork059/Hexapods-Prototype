import math
import sys

# ============================
# USER CONFIG (edit as needed)
# ============================
TARGETS_HIP = [
    (80.0, 0.0, -80.0),
    (70.0, 10.0, -75.0),
    (70.0, -10.0, -75.0),
    (90.0, 0.0, -70.0),
    (90.0, 15.0, -90.0),
    (85.0, -15.0, -90.0),
    (60.0, 0.0, -70.0),
    (60.0, 20.0, -80.0),
    (60.0, -20.0, -80.0),
    (75.0, 0.0, -95.0),
]
LEG_INDEX = 0
# ============================

if ".." not in sys.path:
    sys.path.append("..")

import config
import ik
from util import assert_true


def _finite(*values):
    return all(math.isfinite(float(v)) for v in values)


def test_ik_targets_within_limits():
    channels = config.LEG_SERVOS[LEG_INDEX]
    for target in TARGETS_HIP:
        sol = ik.ik_xyz(*target)
        assert_true(_finite(sol["coxa"], sol["femur"], sol["tibia"]))

        servo_angles = [
            ik.logical_to_servo_angle(channels[0], sol["coxa"]),
            ik.logical_to_servo_angle(channels[1], sol["femur"]),
            ik.logical_to_servo_angle(channels[2], sol["tibia"]),
        ]

        for idx, angle in enumerate(servo_angles):
            ch = channels[idx]
            lo = float(config.SERVO_MIN_A[ch])
            hi = float(config.SERVO_MAX_A[ch])
            assert_true(
                lo <= angle <= hi,
                "Servo %d angle %.2f out of range [%.2f, %.2f]" % (ch, angle, lo, hi),
            )
