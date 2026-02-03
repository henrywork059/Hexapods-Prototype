# config.py
import math

# Hardware Configuration
BLE_NAME = "Hexa24"
UART_ID = 0
UART_BAUD = 115200
UART_TX = 0
UART_RX = 1

# IMU Configuration
I2C0_ID, SDA0, SCL0 = 0, 4, 5
I2C1_ID, SDA1, SCL1 = 1, 2, 3
IMU_ADDR0 = 0x68
IMU_ADDR1 = 0x68
BASELINE_MM = (46.5, 0.0, 20.0)

# Robot Geometry
BODY_DIAMETER = 161.124
BODY_RADIUS = BODY_DIAMETER * 0.5
COXA_L = 34.50
FEMUR_L = 64.00
TIBIA_L = 86.71
MAX_REACH = (FEMUR_L + TIBIA_L) * 0.95
HIP_RADIUS = BODY_RADIUS
NEUTRAL_RADIUS = HIP_RADIUS + COXA_L + 35.0

# Gait Configuration
DUTY = 0.58
BASE_CYCLE_HZ = 1.2
STEP_FWD = 35.0
STEP_SIDE = 25.0
LIFT = 25.0
STANCE_Z0 = -80.0
Z_RANGE = 30.0

# Leg Configuration (hex layout)
MOUNT_ANGLES = [
    math.pi/6, math.pi/2, 5*math.pi/6,
    7*math.pi/6, 3*math.pi/2, 11*math.pi/6
]
TRIPOD_A = (0, 3, 4)
TRIPOD_B = (1, 2, 5)

# Servo Mapping
LEG_SERVOS = [
    (0, 1, 2),    # Leg 1: coxa, femur, tibia
    (3, 4, 5),    # Leg 2
    (6, 7, 8),    # Leg 3
    (9, 10, 11),  # Leg 4
    (12, 13, 14), # Leg 5
    (15, 16, 17), # Leg 6
]
UNUSED_SERVOS = list(range(18, 24))

# Servo Calibration
SERVO_OFFSET = [0] * 24
SERVO_MIN_A = [0] * 24
SERVO_MAX_A = [180] * 24

# Servo Inversion (True = reversed direction)
SERVO_INVERT = [
    False, True, True,    # Leg 1
    False, True, True,    # Leg 2
    False, True, True,    # Leg 3
    False, True, True,    # Leg 4
    False, True, True,    # Leg 5
    False, True, True,    # Leg 6
    False, False, False, False, False, False  # Unused
]

# Control Parameters
LOOP_MS = 20
DEADZONE = 0.04
LINK_TIMEOUT_MS = 600
UART_SPACING_MS = 1
SERVO_MOVE_MS = 70

# BLE Packet Structure
PKT_HEADER = 0xAA
PKT_LEN = 6
FLAG_ENABLE = 0x01
FLAG_CAL_IMU = 0x02
FLAG_STAND = 0x04

# IMU Leveling
LEVEL_ENABLE = True
LEVEL_GAIN = 0.55

# Logging
DEBUG = True
LOG_TO_FILE = True
LOG_FILE = "log.txt"
LOG_MAX_BYTES = 32768
LOG_FLUSH_EVERY = 10