"""Simulation configuration derived from the real robot config.

This module mirrors the real config shape and fills in fields needed by
simulation-only modules (frames, gait, IK). The intent is to keep the
simulation's API parallel to the real control stack while remaining
fully isolated from hardware.
"""

from __future__ import annotations

import importlib
import math
from typing import Iterable, Sequence


try:
    _base = importlib.import_module("config")
except Exception:
    _base = None


def _get(name: str, default):
    if _base is None:
        return default
    return getattr(_base, name, default)


# ---- Core timing ----
DT = float(_get("DT", 0.02))
GAIT_PERIOD = float(_get("GAIT_PERIOD", 0.8))

# ---- Geometry ----
COXA_L = float(_get("COXA_L", 34.5))
FEMUR_L = float(_get("FEMUR_L", 64.0))
TIBIA_L = float(_get("TIBIA_L", 86.71))
MAX_REACH = float(_get("MAX_REACH", (FEMUR_L + TIBIA_L) * 0.95))

BODY_DIAMETER = float(_get("BODY_DIAMETER", 161.124))
BODY_RADIUS = float(_get("BODY_RADIUS", BODY_DIAMETER * 0.5))
HIP_RADIUS = float(_get("HIP_RADIUS", BODY_RADIUS))
NEUTRAL_RADIUS = float(_get("NEUTRAL_RADIUS", HIP_RADIUS + COXA_L + 35.0))

# ---- Gait configuration ----
TRIPOD_A = list(_get("TRIPOD_A", [0, 2, 4]))
TRIPOD_B = list(_get("TRIPOD_B", [1, 3, 5]))

SWING_STYLE = str(_get("SWING_STYLE", "sine"))
SWING_EASE = str(_get("SWING_EASE", "quintic"))
STANCE_EASE = str(_get("STANCE_EASE", "linear"))

SWING_CLEARANCE = float(_get("SWING_CLEARANCE", 25.0))
SWING_CLEARANCE_GAIN = float(_get("SWING_CLEARANCE_GAIN", 0.15))
SWING_CLEARANCE_MAX = float(_get("SWING_CLEARANCE_MAX", 50.0))
CMD_DEADBAND = float(_get("CMD_DEADBAND", 0.5))
STEP_LIMIT_MM = float(_get("STEP_LIMIT_MM", 40.0))

# ---- Servo mapping ----
LEG_SERVOS = list(_get("LEG_SERVOS", [(0, 1, 2), (3, 4, 5), (6, 7, 8), (9, 10, 11), (12, 13, 14), (15, 16, 17)]))
SERVO_INVERT = list(_get("SERVO_INVERT", [False] * 24))
SERVO_OFFSET = list(_get("SERVO_OFFSET", [0.0] * 24))
SERVO_MIN_A = list(_get("SERVO_MIN_A", [0.0] * 24))
SERVO_MAX_A = list(_get("SERVO_MAX_A", [180.0] * 24))

# ---- Body/hip placement ----
MOUNT_ANGLES = list(
    _get(
        "MOUNT_ANGLES",
        [math.pi / 6, math.pi / 2, 5 * math.pi / 6, 7 * math.pi / 6, 3 * math.pi / 2, 11 * math.pi / 6],
    )
)

hip_pos_B = _get("hip_pos_B", None)
if hip_pos_B is None:
    hip_pos_B = [
        (HIP_RADIUS * math.cos(theta), HIP_RADIUS * math.sin(theta), 0.0)
        for theta in MOUNT_ANGLES
    ]

hip_yaw0 = _get("hip_yaw0", None)
if hip_yaw0 is None:
    hip_yaw0 = [math.atan2(y, x) for (x, y, _z) in hip_pos_B]

# ---- Neutral foot pose (hip-local) ----
STANCE_Z0 = float(_get("STANCE_Z0", -80.0))

_NEUTRAL_OFFSET = max(20.0, NEUTRAL_RADIUS - HIP_RADIUS)
FOOT_NEUTRAL_H = [
    (float(_get("FOOT_NEUTRAL_X", _NEUTRAL_OFFSET)), 0.0, STANCE_Z0)
    for _ in range(6)
]


def listify(value: Iterable) -> list:
    return list(value)


def ensure_len(seq: Sequence, length: int, name: str) -> list:
    if len(seq) != length:
        raise ValueError(f"{name} must have length {length} (got {len(seq)})")
    return list(seq)


ensure_len(LEG_SERVOS, 6, "LEG_SERVOS")
ensure_len(hip_pos_B, 6, "hip_pos_B")
ensure_len(hip_yaw0, 6, "hip_yaw0")
