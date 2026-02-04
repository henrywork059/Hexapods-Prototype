# ik.py (PCA9685 I2C or UART)
# Hip-local IK for a 3DOF hexapod leg.
#
# Convention locked to your request:
#   - Hip joint is (0,0,0)
#   - z NEGATIVE is DOWN (under the hip)
#   - IK returns 3 *logical* joint angles (deg): coxa, femur, tibia
#   - Those logical angles are converted to servo angles using:
#       servo = 90 + sign*logical + offset
#     where sign = -1 if SERVO_INVERT[channel] else +1
#
# Output driver:
#   - If config.SERVO_DRIVER == "pca9685" -> uses servo_pca.py
#   - Otherwise -> uses servo_uart.py
#
# Required config variables:
#   COXA_L, FEMUR_L, TIBIA_L
#   LEG_SERVOS (6x tuples)
#   SERVO_INVERT, SERVO_OFFSET
#   SERVO_MIN_A, SERVO_MAX_A   (optional but recommended)

import math
import time

# --- config import (supports either config.py or config_pca.py) ---
try:
    import config
except ImportError:
    import config_pca as config  # fallback if you didn't rename config_pca.py

# --- servo output module selection ---
_SERVO_DRIVER = getattr(config, "SERVO_DRIVER", "uart")
try:
    if _SERVO_DRIVER == "pca9685":
        import servo_pca as su
    else:
        import servo_uart as su
except ImportError as e:
    raise ImportError(
        "Servo driver import failed. "
        "Check that servo_pca.py or servo_uart.py exists and matches config.SERVO_DRIVER."
    ) from e


EPS = 1e-9

# Geometry from your real robot config
COXA_L = float(getattr(config, "COXA_L", 0.0))
FEMUR_L = float(config.FEMUR_L)
TIBIA_L = float(config.TIBIA_L)

# Optional logical joint limits (deg) for safety / realism
COXA_LIM = (-60.0, 60.0)
FEMUR_LIM = (-90.0, 90.0)
TIBIA_LIM = (-140.0, 0.0)  # tibia is negative when folding (knee-up)

# Reach limits (avoid singularities + unrealistic commands)
REACH_MIN = 10.0
REACH_MAX = float(getattr(config, "MAX_REACH", (FEMUR_L + TIBIA_L) * 0.95))


def _sleep_ms(ms: int):
    """Cross-platform sleep_ms for MicroPython + CPython."""
    try:
        time.sleep_ms(int(ms))
    except AttributeError:
        time.sleep(float(ms) / 1000.0)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def rad2deg(r):
    return r * 180.0 / math.pi


def deg2rad(d):
    return d * math.pi / 180.0


def logical_to_servo_angle(channel, logical_deg):
    """Convert logical joint angle (deg) to servo absolute angle (deg)."""
    sign = -1.0 if config.SERVO_INVERT[channel] else +1.0
    off = float(config.SERVO_OFFSET[channel]) if hasattr(config, "SERVO_OFFSET") else 0.0
    servo = 90.0 + sign * float(logical_deg) + off

    # Apply per-servo min/max if configured
    if hasattr(config, "SERVO_MIN_A") and hasattr(config, "SERVO_MAX_A"):
        servo = clamp(servo, float(config.SERVO_MIN_A[channel]), float(config.SERVO_MAX_A[channel]))

    return servo


def ik_xyz(x, y, z):
    """Solve IK for ONE leg in hip-local coordinates.

    Inputs (mm):
      x, y, z  (z negative under hip)

    Returns:
      dict with logical angles (deg): coxa, femur, tibia
      plus debug terms: r, z_down, d, clamped_reach

    Branch:
      Knee-up so the leg looks like /\ (not \/).
    """
    x = float(x)
    y = float(y)
    z = float(z)

    # 1) Coxa yaw
    coxa = rad2deg(math.atan2(y, x))
    coxa = clamp(coxa, COXA_LIM[0], COXA_LIM[1])

    # Project distance in the coxa heading direction
    r_xy = math.hypot(x, y)

    # After coxa link
    r = r_xy - COXA_L
    if r < 1.0:
        r = 1.0

    # z convention: negative is down -> z_down is positive downward
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

    # 2-link IK in r-z plane
    alpha = math.atan2(z_down, r)

    cos_beta = (a * a + d * d - b * b) / (2.0 * a * d + EPS)
    cos_beta = clamp(cos_beta, -1.0, 1.0)
    beta = math.acos(cos_beta)

    # Knee-up branch
    femur = rad2deg(alpha - beta)

    cos_gamma = (a * a + b * b - d * d) / (2.0 * a * b + EPS)
    cos_gamma = clamp(cos_gamma, -1.0, 1.0)
    gamma = math.acos(cos_gamma)

    # Tibia negative when folding
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
    """Move one leg's end-effector to (x,y,z) in hip-local coordinates (mm).

    Sends the 3 servos as one pose update.
    """
    leg_idx = int(leg_idx)
    sc, sf, st = config.LEG_SERVOS[leg_idx]

    sol = ik_xyz(x, y, z)

    a_sc = logical_to_servo_angle(sc, sol["coxa"])
    a_sf = logical_to_servo_angle(sf, sol["femur"])
    a_st = logical_to_servo_angle(st, sol["tibia"])

    su.set_pose([(sc, a_sc), (sf, a_sf), (st, a_st)], t_ms=t_ms)
    return sol


def move_all_legs_xyz(targets, t_ms=None):
    """Move all 6 legs in one pose update.

    targets: list/tuple length 6 of (x,y,z) hip-local for each leg.
    """
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


# Quick sanity run (safe-ish): centers then wiggles leg 0 a bit
if __name__ == "__main__":
    print("Centering all servos...")
    try:
        su.center_all(t_ms=300)
    except AttributeError:
        # Some drivers may not have this helper; ignore.
        pass
    _sleep_ms(600)

    print("Test leg 0 IK move...")
    move_leg_xyz(0, 80, 0, -80, t_ms=220)
    _sleep_ms(500)
    move_leg_xyz(0, 60, 0, -70, t_ms=220)
    _sleep_ms(500)
    move_leg_xyz(0, 80, 0, -80, t_ms=220)
    _sleep_ms(500)

    print("Done.")
