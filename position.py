# from machine import UART, Pin
# import time
# 
# # ======================
# # UART CONFIG
# # ======================
# uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
# 
# # ======================
# # JOINT CONFIG
# # ======================
# # 0 = coxa (normal)
# # 1 = femur (inverted)
# # 2 = tibia (inverted)
# 
# JOINT_TYPE = [
#     0,1,2,  0,1,2,  0,1,2,
#     0,1,2,  0,1,2,  0,1,2,
# ]
# 
# JOINT_SIGN = {
#     0: +1,
#     1: -1,
#     2: -1,
# }
# 
# # ======================
# # SERVO HELPERS
# # ======================
# def angle_to_pwm(angle):
#     angle = max(30, min(150, int(angle)))
#     return int(500 + angle * 2000 / 180)
# 
# def send_servo(idx, angle, t_ms=1000):
#     pwm = angle_to_pwm(angle)
#     cmd = "#%03dP%04dT%04d!" % (idx, pwm, t_ms)
#     uart.write(cmd.encode())
#     time.sleep_ms(15)
# 
# def send_joint(idx, logical_deg, t_ms=1000):
#     jt = JOINT_TYPE[idx]
#     sign = JOINT_SIGN[jt]
#     servo_angle = 90 + sign * logical_deg
#     send_servo(idx, servo_angle, t_ms)
# 
# # ======================
# # LEG MAP
# # ======================
# LEG = [
#     (0, 1, 2),
#     (3, 4, 5),
#     (6, 7, 8),
#     (9,10,11),
#     (12,13,14),
#     (15,16,17),
# ]
# 
# # ======================
# # >>> DIRECT STANDING ANGLES <<<
# # ======================
# # LOGICAL angles (relative to 90)
# # Tune THESE by eye
# 
# STAND_POSE = [
#     # (coxa, femur, tibia)
#     (0, -10, -180),   # Leg 0
#     (0, -10, -180),   # Leg 1
#     (0, -10, -180),   # Leg 2
#     (0, -10, -180),   # Leg 3
#     (0, -10, -180),   # Leg 4
#     (0, -10, -180),   # Leg 5
# ]
# 
# # ======================
# # EXECUTION
# # ======================
# # print("\nStarting in 3 seconds...")
# # time.sleep(3)
# 
# print("Zero all servos")
# for ch in range(18):
#     send_servo(ch, 90, 1000)
# 
# time.sleep(2)
# 
# print("Applying standing pose (simultaneous)")
# 
# T_STAND = 300  # ms — slow and safe
# 
# # Send ALL commands first
# for leg in range(6):
#     sc, sf, st = LEG[leg]
#     coxa, femur, tibia = STAND_POSE[leg]
# 
#     send_joint(sc, coxa, T_STAND)
#     send_joint(sf, femur, T_STAND)
#     send_joint(st, tibia, T_STAND)
# 
# # Now wait for motion to complete
# time.sleep_ms(T_STAND + 200)
# 
# print("Standing pose reached.")
# 
# while True:
#     time.sleep(5)
#


from machine import UART, Pin
import time

# ======================
# UART CONFIG
# ======================
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# ======================
# LOW-LEVEL SERVO SEND
# ======================
def angle_to_pwm(angle):
    angle = max(30, min(150, int(angle)))
    return int(500 + angle * 2000 / 180)

def send_servo(idx, angle, t_ms):
    pwm = angle_to_pwm(angle)
    cmd = "#%03dP%04dT%04d!" % (idx, pwm, t_ms)
    uart.write(cmd.encode())
    time.sleep_ms(10)

# ======================
# INIT POSE (ABSOLUTE)
# ======================
INIT_POSE = [
    90,105,150,   # leg 0
    90,105,150,   # leg 1
    90,105,150,   # leg 2
    90,105,150,   # leg 3
    90,105,150,   # leg 4
    90,105,150,   # leg 5
]

# ======================
# EXECUTION
# ======================
print("Zeroing all servos")
for ch in range(18):
    send_servo(ch, 90, 500)

print("Waiting 3 seconds...")
time.sleep(3)

print("Moving to INIT pose (simultaneous)")
T_INIT = 100

for ch in range(18):
    send_servo(ch, INIT_POSE[ch], T_INIT)

time.sleep_ms(T_INIT + 200)

print("Init pose reached. Holding.")

while True:
    time.sleep(5)
