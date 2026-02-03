# config.py
# Hexapod configuration (geometry + calibration + servo mapping)
# Updated for an I2C PCA9685-style PWM servo driver.
#
# This file is the "single source of truth" for your robot.
# Put everything you want to tweak in the USER-EDIT SECTION.
#
# Conventions:
# - Body frame (B): origin at body center, +x forward, +y left, +z up
# - Each leg uses a hip-local IK frame:
#     +x: radial outward from hip
#     +y: left (leg-local)
#     +z: up  (so under-hip is negative z)
#
# Notes:
# - PCA9685 is 16 channels per board. For 18 servos you usually need TWO boards.
# - Your IK code expects config variables: COXA_L/FEMUR_L/TIBIA_L, LEG_SERVOS,
#   SERVO_INVERT/SERVO_OFFSET, SERVO_MIN_A/SERVO_MAX_A, hip_pos_B, hip_yaw0.

import math

# ============================================================
# USER-EDIT SECTION (change only things in this section)
# ============================================================

# --- Timing ---
DT = 0.02                 # control loop tick (s). 0.02 => 50 Hz
SERVO_MOVE_MS = 120       # default move time (ms) used by motion commands

# --- Driver selection ---
SERVO_DRIVER = "pca9685"

# --- I2C pins (MicroPython) ---
# Common RP2040/Pico defaults: I2C0 on SDA=0, SCL=1
I2C_ID = 0
I2C_SDA = 0
I2C_SCL = 1
I2C_FREQ = 400_000

# --- PCA9685 boards on the bus ---
# Two boards give you 32 channels total.
PCA_ADDRS = [0x40, 0x41]
PCA_FREQ_HZ = 50

# --- Servo pulse range (microseconds) ---
SERVO_US_MIN = 500
SERVO_US_MAX = 2500

# --- Channel / leg mapping ---
# Logical channels are 0..23 (legacy from the 24ch board). You will use 0..17 for 18 servos.
NUM_LOGICAL_CHANNELS = 24
NUM_USED_SERVOS = 18

# Leg index order is your choice, but must be consistent everywhere.
# Default: legs 0..5 each use 3 consecutive channels: (coxa, femur, tibia).
LEG_SERVOS = [
    (0,  1,  2),   # leg 0
    (3,  4,  5),   # leg 1
    (6,  7,  8),   # leg 2
    (9,  10, 11),  # leg 3
    (12, 13, 14),  # leg 4
    (15, 16, 17),  # leg 5
]

# --- Calibration (direction + offsets + clamps) ---
# These are applied by IK when converting logical angles to servo absolute angles.
#
# Defaults below are safe and symmetric. Only override what you need.
SERVO_INVERT_OVERRIDES = {
    # example: 0: True,
}
SERVO_OFFSET_OVERRIDES = {
    # example: 0: +5.0,   # degrees
}

# Global clamp defaults (deg). Tighten during bring-up; widen later if safe.
SERVO_MIN_A_DEFAULT = 30.0
SERVO_MAX_A_DEFAULT = 150.0

# Optional per-channel clamp overrides
SERVO_MIN_A_OVERRIDES = {
    # example: 0: 20.0,
}
SERVO_MAX_A_OVERRIDES = {
    # example: 0: 160.0,
}

# --- Leg geometry (mm) ---
# Start with your sim numbers if unsure.
COXA_L  = 0.0     # hip yaw link length (mm). Set >0 if your coxa link is real.
FEMUR_L = 80.0    # upper link (mm)
TIBIA_L = 100.0   # lower link (mm)
MAX_REACH = (FEMUR_L + TIBIA_L) * 0.95

# --- Body geometry (mm): hip positions in body frame ---
# !!! YOU MUST EDIT THESE to match your chassis !!!
BODY_X = 70.0
BODY_Y = 55.0
HIP_Z  = 0.0

# Example leg order: 0 FR, 1 MR, 2 BR, 3 FL, 4 ML, 5 BL (edit if different)
hip_pos_B = [
    (+BODY_X, -BODY_Y, HIP_Z),  # leg 0
    (0.0,     -BODY_Y, HIP_Z),  # leg 1
    (-BODY_X, -BODY_Y, HIP_Z),  # leg 2
    (+BODY_X, +BODY_Y, HIP_Z),  # leg 3
    (0.0,     +BODY_Y, HIP_Z),  # leg 4
    (-BODY_X, +BODY_Y, HIP_Z),  # leg 5
]

# ============================================================
# DERIVED SECTION (do not edit unless you know why)
# ============================================================

# Map each logical channel -> (board_index, pca_channel)
# Default mapping:
# - logical 0..15  => board 0 ch 0..15
# - logical 16..23 => board 1 ch 0..7
LOGICAL_TO_PCA = []
for ch in range(NUM_LOGICAL_CHANNELS):
    if ch < 16:
        LOGICAL_TO_PCA.append((0, ch))
    else:
        LOGICAL_TO_PCA.append((1, ch - 16))

# Expand calibration dicts into full lists (size NUM_LOGICAL_CHANNELS)
SERVO_INVERT = [False] * NUM_LOGICAL_CHANNELS
for k, v in SERVO_INVERT_OVERRIDES.items():
    SERVO_INVERT[int(k)] = bool(v)

SERVO_OFFSET = [0.0] * NUM_LOGICAL_CHANNELS
for k, v in SERVO_OFFSET_OVERRIDES.items():
    SERVO_OFFSET[int(k)] = float(v)

SERVO_MIN_A = [float(SERVO_MIN_A_DEFAULT)] * NUM_LOGICAL_CHANNELS
for k, v in SERVO_MIN_A_OVERRIDES.items():
    SERVO_MIN_A[int(k)] = float(v)

SERVO_MAX_A = [float(SERVO_MAX_A_DEFAULT)] * NUM_LOGICAL_CHANNELS
for k, v in SERVO_MAX_A_OVERRIDES.items():
    SERVO_MAX_A[int(k)] = float(v)

# Hip mount yaw (rad): aligns body-frame radial direction to hip-local +x
hip_yaw0 = [math.atan2(y, x) for (x, y, _z) in hip_pos_B]

# ============================================================
# LEGACY UART FIELDS (kept so old imports don't crash)
# ============================================================
UART_ID = 0
UART_BAUD = 115200
UART_TX = 0
UART_RX = 1
UART_SPACING_MS = 1
