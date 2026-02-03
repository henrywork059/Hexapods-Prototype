# config.py
# Hexapod configuration (geometry + calibration + servo mapping)
# Updated for an I2C PCA9685-style PWM servo driver.
#
# This file is intentionally shaped to satisfy the constants expected by:
#   - ik_fixed.py (COXA_L/FEMUR_L/TIBIA_L, LEG_SERVOS, SERVO_INVERT/OFFSET, SERVO_MIN_A/MAX_A)
#   - frames.py / gait code (hip_pos_B, hip_yaw0)
# while replacing the old UART servo-board settings with PCA I2C settings.
#
# IMPORTANT:
# - A single PCA9685 has 16 channels. For 18 servos you usually need TWO boards.
#   Default below assumes addresses 0x40 and 0x41.
# - If you truly have only ONE 16ch board, you can control at most 16 servos.

import math

# -----------------------------
# 0) Runtime timing defaults
# -----------------------------
DT = 0.02                 # control loop tick (s). 0.02 => 50 Hz
SERVO_MOVE_MS = 120       # default move time for commands (ms)

# -----------------------------
# 1) Servo driver selection
# -----------------------------
SERVO_DRIVER = "pca9685"   # used by your servo output module (e.g., servo_pca.py)

# -----------------------------
# 2) I2C (PCA9685) settings
# -----------------------------
# MicroPython pins:
# - RP2040/Pico common defaults: SDA=0, SCL=1 on I2C0
# Change these to match your wiring.
I2C_ID = 0
I2C_SDA = 0
I2C_SCL = 1
I2C_FREQ = 400_000

# PCA boards on the bus (two boards gives you 32 channels total)
PCA_ADDRS = [0x40, 0x41]
PCA_FREQ_HZ = 50          # servos expect ~50 Hz

# Servo pulse range (microseconds). Tune per servo if needed.
SERVO_US_MIN = 500
SERVO_US_MAX = 2500

# Optional: for some PCA boards you may need to invert the PWM output.
# Leave False unless you know you need it.
PCA_INVERT_OUTPUT = False

# -----------------------------
# 3) Channel mapping
# -----------------------------
# Your code (IK / gait) works with a "logical servo channel" index.
# We keep the old convention of channels 0..23 (because earlier code used a 24ch board),
# but you will use only 0..17 for an 18-servo hexapod.
NUM_LOGICAL_CHANNELS = 24
NUM_USED_SERVOS = 18

# Map each logical channel -> (board_index, pca_channel)
# - logical 0..15  => board 0 ch 0..15
# - logical 16..23 => board 1 ch 0..7
# Adjust if your wiring is different.
LOGICAL_TO_PCA = []
for ch in range(NUM_LOGICAL_CHANNELS):
    if ch < 16:
        LOGICAL_TO_PCA.append((0, ch))
    else:
        LOGICAL_TO_PCA.append((1, ch - 16))

# -----------------------------
# 4) Leg servo mapping (3 servos per leg)
# -----------------------------
# Leg index order is YOUR choice, but must be consistent everywhere.
# Default follows the earlier doc template: legs 0..5 each use 3 consecutive channels.
LEG_SERVOS = [
    (0,  1,  2),   # leg 0: (coxa, femur, tibia)
    (3,  4,  5),   # leg 1
    (6,  7,  8),   # leg 2
    (9,  10, 11),  # leg 3
    (12, 13, 14),  # leg 4
    (15, 16, 17),  # leg 5
]

# -----------------------------
# 5) Servo calibration (direction, offsets, clamps)
# -----------------------------
# Used by ik_fixed.py: servo = 90 + sign*logical + offset
SERVO_INVERT = [False] * NUM_LOGICAL_CHANNELS
SERVO_OFFSET = [0.0] * NUM_LOGICAL_CHANNELS

# Per-servo safety clamp in degrees (recommended while testing).
# If you are confident, widen toward (0,180) per channel.
SERVO_MIN_A = [30.0] * NUM_LOGICAL_CHANNELS
SERVO_MAX_A = [150.0] * NUM_LOGICAL_CHANNELS

# -----------------------------
# 6) Leg geometry (mm)
# -----------------------------
# Fill these with your real measurements.
# If you previously used upper=80mm and lower=100mm in the sim, start with that:
COXA_L  = 0.0     # mm (hip yaw link length). Measure if non-zero.
FEMUR_L = 80.0    # mm
TIBIA_L = 100.0   # mm

# Optional overall reach clamp used by ik_fixed.py
# (defaults to 0.95*(FEMUR_L+TIBIA_L) if not provided)
MAX_REACH = (FEMUR_L + TIBIA_L) * 0.95

# -----------------------------
# 7) Body geometry (mm) for body->hip transforms
# -----------------------------
# Body frame (B): origin at body center, +x forward, +y left, +z up.
# hip_pos_B[i] is the hip joint position of leg i in body frame.
#
# !!! YOU MUST EDIT THESE to match your chassis !!!
# Below is a symmetric placeholder layout for a typical tabletop hexapod.
BODY_X = 70.0   # forward/back hip spacing from center (mm)
BODY_Y = 55.0   # left/right hip spacing from center (mm)
HIP_Z  = 0.0    # hip height offset in body frame (mm)

# Example leg order (edit if your leg indices are different):
# 0: front-right, 1: mid-right, 2: back-right, 3: front-left, 4: mid-left, 5: back-left
hip_pos_B = [
    (+BODY_X, -BODY_Y, HIP_Z),  # leg 0
    (0.0,     -BODY_Y, HIP_Z),  # leg 1
    (-BODY_X, -BODY_Y, HIP_Z),  # leg 2
    (+BODY_X, +BODY_Y, HIP_Z),  # leg 3
    (0.0,     +BODY_Y, HIP_Z),  # leg 4
    (-BODY_X, +BODY_Y, HIP_Z),  # leg 5
]

# Mount yaw per hip (rad): aligns body-frame radial direction to hip-local +x
# Common definition from the doc: atan2(y, x)
hip_yaw0 = [math.atan2(y, x) for (x, y, _z) in hip_pos_B]

# -----------------------------
# 8) Legacy UART fields (kept so old imports don't crash)
# -----------------------------
# If you are no longer using the UART servo board, these are unused.
UART_ID = 0
UART_BAUD = 115200
UART_TX = 0
UART_RX = 1
UART_SPACING_MS = 1
