"""Hip-local IK for simulation (same interface as real ik.py)."""

from __future__ import annotations

import math

from . import config
from . import virtual_servo as su

EPS = 1e-9

COXA_L = float(config.COXA_L)
FEMUR_L = float(config.FEMUR_L)
TIBIA_L = float(config.TIBIA_L)

COXA_LIM = (-60.0, 60.0)
FEMUR_LIM = (-90.0, 90.0)
TIBIA_LIM = (-140.0, 0.0)

REACH_MIN = 10.0
REACH_MAX = float(config.MAX_REACH)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def rad2deg(r):
    return r * 180.0 / math.pi


def logical_to_servo_angle(channel, logical_deg):
    sign = -1.0 if config.SERVO_INVERT[channel] else +1.0
    off = float(config.SERVO_OFFSET[channel]) if hasattr(config, "SERVO_OFFSET") else 0.0
    servo = 90.0 + sign * float(logical_deg) + off

    if hasattr(config, "SERVO_MIN_A") and hasattr(config, "SERVO_MAX_A"):
        servo = clamp(servo, float(config.SERVO_MIN_A[channel]), float(config.SERVO_MAX_A[channel]))

    return servo


def ik_xyz(x, y, z):
    x = float(x)
    y = float(y)
    z = float(z)

    coxa = rad2deg(math.atan2(y, x))
    coxa = clamp(coxa, COXA_LIM[0], COXA_LIM[1])

    r_xy = math.hypot(x, y)
    r = r_xy - COXA_L
    if r < 1.0:
        r = 1.0

    z_down = -z
    d = math.hypot(r, z_down)
    clamped_reach = False
    if d < REACH_MIN:
        d = REACH_MIN
        clamped_reach = True
    if d > REACH_MAX:
        d = REACH_MAX
        clamped_reach = True

    a = FEMUR_L
    b = TIBIA_L

    alpha = math.atan2(z_down, r)

    cos_beta = (a * a + d * d - b * b) / (2.0 * a * d + EPS)
    cos_beta = clamp(cos_beta, -1.0, 1.0)
    beta = math.acos(cos_beta)

    femur = rad2deg(alpha - beta)

    cos_gamma = (a * a + b * b - d * d) / (2.0 * a * b + EPS)
    cos_gamma = clamp(cos_gamma, -1.0, 1.0)
    gamma = math.acos(cos_gamma)

    tibia = rad2deg(-(math.pi - gamma))

    femur = clamp(femur, FEMUR_LIM[0], FEMUR_LIM[1])
    tibia = clamp(tibia, TIBIA_LIM[0], TIBIA_LIM[1])

    return {
        "coxa": coxa,
        "femur": femur,
        "tibia": tibia,
        "r": r,
        "z_down": z_down,
        "d": d,
        "clamped_reach": clamped_reach,
    }


def move_leg_xyz(leg_idx, x, y, z, t_ms=None):
    leg_idx = int(leg_idx)
    sc, sf, st = config.LEG_SERVOS[leg_idx]

    sol = ik_xyz(x, y, z)

    a_sc = logical_to_servo_angle(sc, sol["coxa"])
    a_sf = logical_to_servo_angle(sf, sol["femur"])
    a_st = logical_to_servo_angle(st, sol["tibia"])

    su.set_pose([(sc, a_sc), (sf, a_sf), (st, a_st)], t_ms=t_ms)
    return sol


def move_all_legs_xyz(targets, t_ms=None):
    if len(targets) != 6:
        raise ValueError("targets must be length 6: [(x,y,z), ...]")

    pairs = []
    sols = []
    for i in range(6):
        x, y, z = targets[i]
        sc, sf, st = config.LEG_SERVOS[i]
        sol = ik_xyz(x, y, z)
        sols.append(sol)
        pairs.append((sc, logical_to_servo_angle(sc, sol["coxa"])))
        pairs.append((sf, logical_to_servo_angle(sf, sol["femur"])))
        pairs.append((st, logical_to_servo_angle(st, sol["tibia"])))

    su.set_pose(pairs, t_ms=t_ms)
    return sols
