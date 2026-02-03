# gait_tripod.py
import math
import time

# ==========================
# GAIT CONSTANTS (LOGICAL)
# ==========================
NEUTRAL_X = 50
NEUTRAL_Z = -50     # +Z = DOWN
LIFT_Z    = 50
STEP_LEN  = 40

SWING_TIME  = 200   # ms
STANCE_TIME = 200   # ms

TRIPOD_A = (0, 3, 4)
TRIPOD_B = (1, 2, 5)

# ==========================
# LEG GEOMETRY (mm)
# ==========================
FEMUR = 80
TIBIA = 100

# ==========================
# PLANAR IK (LOGICAL OUTPUT)
# ==========================
def ik_leg(x, z):
    """
    Coordinate frame:
        +x forward
        +z downward

    Returns:
        femur_logical_deg
        tibia_logical_deg
    """

    d = math.sqrt(x*x + z*z)
    d = min(max(d, 10), FEMUR + TIBIA - 1)

    # Femur
    alpha = math.atan2(z, x)
    beta = math.acos(
        (FEMUR*FEMUR + d*d - TIBIA*TIBIA) /
        (2 * FEMUR * d)
    )
    femur = alpha + beta          # + = leg down

    # Tibia
    gamma = math.acos(
        (FEMUR*FEMUR + TIBIA*TIBIA - d*d) /
        (2 * FEMUR * TIBIA)
    )
    tibia = -(math.pi - gamma)    # negative = knee fold down

    return math.degrees(femur), math.degrees(tibia)

# ==========================
# TRIPOD GAIT ENGINE
# ==========================
class TripodGait:
    def __init__(self, send_joint_fn):
        self.send_joint = send_joint_fn

    def set_leg(self, leg, x, z, t_ms):
        femur, tibia = ik_leg(x, z)
        base = leg * 3

        # Coxa fixed straight for now
        self.send_joint(base + 0, 0,     t_ms)
        self.send_joint(base + 1, femur, t_ms)
        self.send_joint(base + 2, tibia, t_ms)

    def step(self, swing, stance):
        # Lift swing legs
        for l in swing:
            self.set_leg(l, NEUTRAL_X - STEP_LEN/2, LIFT_Z, SWING_TIME)
        time.sleep_ms(SWING_TIME)

        # Swing forward
        for l in swing:
            self.set_leg(l, NEUTRAL_X + STEP_LEN/2, LIFT_Z, SWING_TIME)
        time.sleep_ms(SWING_TIME)

        # Place down
        for l in swing:
            self.set_leg(l, NEUTRAL_X + STEP_LEN/2, NEUTRAL_Z, SWING_TIME)
        time.sleep_ms(SWING_TIME)

        # Move stance legs backward
        for l in stance:
            self.set_leg(l, NEUTRAL_X - STEP_LEN/2, NEUTRAL_Z, STANCE_TIME)
        time.sleep_ms(STANCE_TIME)

    def walk_once(self):
        self.step(TRIPOD_B, TRIPOD_A)
        self.step(TRIPOD_A, TRIPOD_B)