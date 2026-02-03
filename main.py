# # main.py
# import time
# from ble_controller import BLEUART
# from servo_controller import ServoController
# from gait_tripod import TripodGait
# 
# # ----------------------------
# # INIT
# # ----------------------------
# ble = BLEUART("Hexa24")
# servo = ServoController()
# gait = TripodGait(servo)
# 
# print("Hexapod ready")
# 
# # safety
# last_cmd = time.ticks_ms()
# WALK_TIMEOUT = 600   # ms
# 
# walking = False
# 
# # ----------------------------
# # MAIN LOOP (NON-BLOCKING)
# # ----------------------------
# while True:
#     ble.poll()
# 
#     pkt = ble.get_packet()
#     if pkt:
#         last_cmd = time.ticks_ms()
# 
#         vx = pkt[1]      # forward
#         flags = pkt[4]
# 
#         walking = (flags & 0x01) != 0
# 
#     # fail-safe
#     if time.ticks_diff(time.ticks_ms(), last_cmd) > WALK_TIMEOUT:
#         walking = False
# 
#     if walking:
#         gait.walk_once()
#     else:
#         time.sleep_ms(20)



# # main.py  (BLE DEBUG)
# import time
# from ble_controller import BLEUART
# 
# ble = BLEUART("Hexa24")
# 
# print("BLE debug running")
# 
# while True:
#     ble.poll()
#     pkt = ble.get_packet()
#     if pkt:
#         print("PKT:", list(pkt))
#     time.sleep_ms(10)

# main.py  (SERVO DEBUG)
# import time
# from servo_controller import ServoController
# 
# servo = ServoController()
# 
# print("Servo test")
# 
# while True:
#     servo.send_servo_command(1, 60, 30)
#     time.sleep(1)
#     servo.send_servo_command(1, 120, 30)
#     time.sleep(1)


# # main.py
# from machine import UART, Pin
# import time
# from gait_tripod import TripodGait, NEUTRAL_X, NEUTRAL_Z, LIFT_Z
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
# def send_servo(idx, angle, t_ms):
#     pwm = angle_to_pwm(angle)
#     cmd = "#%03dP%04dT%04d!" % (idx, pwm, t_ms)
#     uart.write(cmd.encode())
#     time.sleep_ms(15)
# 
# def send_joint(idx, logical_deg, t_ms):
#     jt = JOINT_TYPE[idx]
#     sign = JOINT_SIGN[jt]
#     servo_angle = 90 + sign * logical_deg
#     send_servo(idx, servo_angle, t_ms)
# 
# # ======================
# # STARTUP SAFETY
# # ======================
# print("\nStarting in 3 seconds...")
# time.sleep(3)
# 
# print("Zeroing all servos")
# for ch in range(18):
#     send_servo(ch, 90, 300)
# 
# time.sleep(2)
# 
# # ======================
# # INIT GAIT (SAFE WRAP)
# # ======================
# gait = TripodGait(lambda idx, deg, t: send_joint(idx, deg, t))
# 
# # ======================
# # STANDING POSE
# # ======================
# print("Standing")
# for leg in range(6):
#     gait.set_leg(leg, NEUTRAL_X, NEUTRAL_Z, 300)
# 
# time.sleep(2)
