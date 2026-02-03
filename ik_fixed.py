# ik_fixed.py
# Inverse kinematics using config + servo_uart
# Coordinate: hip at (0,0,0), z NEGATIVE is down

import math
import config
from servo_uart import set_pose, set_time_ms

FEMUR_L = 80.0
TIBIA_L = 100.0

SAFE_MIN = 30.0
SAFE_MAX = 150.0

USE_KNEE_COMPLEMENT = True

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def safe_acos(x):
    return math.acos(clamp(x, -1.0, 1.0))

def servo_sign(sid):
    if hasattr(config, "SERVO_INVERT") and config.SERVO_INVERT[int(sid)]:
        return -1.0
    return 1.0

def servo_offset(sid):
    if hasattr(config, "SERVO_OFFSET"):
        return float(config.SERVO_OFFSET[int(sid)])
    return 0.0

def servo_limits(sid):
    lo = SAFE_MIN
    hi = SAFE_MAX
    if hasattr(config, "SERVO_MIN_A") and hasattr(config, "SERVO_MAX_A"):
        lo = max(lo, config.SERVO_MIN_A[int(sid)])
        hi = min(hi, config.SERVO_MAX_A[int(sid)])
    return lo, hi

def logical_to_servo(sid, logical_deg):
    s = 90 + servo_sign(sid) * logical_deg + servo_offset(sid)
    lo, hi = servo_limits(sid)
    return clamp(s, lo, hi)

def ik_xyz_logical(x, y, z):
    z_down = -z
    r = math.sqrt(x*x + y*y)
    d = math.sqrt(r*r + z_down*z_down)

    d = clamp(d, abs(FEMUR_L - TIBIA_L) + 5, FEMUR_L + TIBIA_L - 5)

    coxa = math.degrees(math.atan2(y, x))

    c2 = (d*d - FEMUR_L*FEMUR_L - TIBIA_L*TIBIA_L) / (2 * FEMUR_L * TIBIA_L)
    theta2 = safe_acos(c2)

    alpha = math.atan2(z_down, r)
    cb = (FEMUR_L*FEMUR_L + d*d - TIBIA_L*TIBIA_L) / (2 * FEMUR_L * d)
    beta = safe_acos(cb)

    femur = math.degrees(alpha + beta)
    knee = math.degrees(theta2)
    tibia = 180 - knee if USE_KNEE_COMPLEMENT else knee

    return coxa, femur, tibia

def move_leg_xyz(leg_idx, x, y, z, t_ms=250):
    coxa_id, femur_id, tibia_id = config.LEG_SERVOS[int(leg_idx)]

    coxa_l, femur_l, tibia_l = ik_xyz_logical(x, y, z)

    coxa_s  = logical_to_servo(coxa_id, coxa_l)
    femur_s = logical_to_servo(femur_id, femur_l)
    tibia_s = logical_to_servo(tibia_id, tibia_l)

    set_time_ms(t_ms)
    set_pose([
        (coxa_id,  coxa_s),
        (femur_id, femur_s),
        (tibia_id, tibia_s),
    ])

    return (coxa_s, femur_s, tibia_s)