# servo_uart.py
# UART helper for YH-24 servo controller (multi-servo packets)

from machine import UART, Pin
import time

_uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
_default_time_ms = 250

def set_time_ms(t_ms):
    global _default_time_ms
    _default_time_ms = int(t_ms)

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def angle_to_pwm(angle):
    angle = clamp(int(angle), 30, 150)
    return int(500 + angle * 2000 / 180)

def set_pose(pairs):
    cmds = []
    for sid, ang in pairs:
        pwm = angle_to_pwm(ang)
        cmds.append("#%03dP%04dT%04d!" % (sid, pwm, _default_time_ms))
    packet = "{" + "".join(cmds) + "}"
    _uart.write(packet.encode())
    time.sleep_ms(2)