# servo_pca.py
# PCA9685 I2C PWM servo driver with a servo_uart-compatible API (timing removed).
#
# Exports:
#   init()
#   set_servo(ch, angle_deg, t_ms=None)           # t_ms is accepted but ignored
#   set_pose([(ch, angle_deg), ...], t_ms=None)   # t_ms is accepted but ignored
#   center_all(t_ms=None, channels=range(18))     # t_ms is accepted but ignored
#
# Notes:
# - PCA9685 does NOT have built-in motion timing.
# - This driver now ALWAYS writes target PWM immediately (non-blocking).
# - Any smoothing / trajectory timing should be done in your higher-level control loop.

import time
import config_pca as config  # uses config_pca.py

try:
    from machine import I2C, Pin
except ImportError:
    I2C = None
    Pin = None

# PCA9685 registers
_MODE1 = 0x00
_MODE2 = 0x01
_PRESCALE = 0xFE
_LED0_ON_L = 0x06

_OSC_HZ = 25_000_000  # typical PCA9685 internal oscillator

_i2c = None
_pcas = []


def _sleep_ms(ms):
    try:
        time.sleep_ms(int(ms))
    except AttributeError:
        time.sleep(float(ms) / 1000.0)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PCA9685:
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = int(addr)

    def read8(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def write8(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([int(val) & 0xFF]))

    def set_pwm(self, channel, on, off):
        c = int(channel)
        base = _LED0_ON_L + 4 * c
        data = bytes([
            int(on) & 0xFF,
            (int(on) >> 8) & 0xFF,
            int(off) & 0xFF,
            (int(off) >> 8) & 0xFF,
        ])
        self.i2c.writeto_mem(self.addr, base, data)

    def set_freq(self, freq_hz):
        freq = float(freq_hz)
        prescale = int(round(_OSC_HZ / (4096.0 * freq) - 1.0))
        prescale = int(clamp(prescale, 3, 255))

        old_mode = self.read8(_MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10  # SLEEP=1

        self.write8(_MODE1, sleep_mode)
        _sleep_ms(1)
        self.write8(_PRESCALE, prescale)
        _sleep_ms(1)

        # Wake + enable auto-increment
        mode1 = (old_mode & 0xEF) | 0x20  # ensure SLEEP=0, AI=1
        self.write8(_MODE1, mode1)
        _sleep_ms(1)
        self.write8(_MODE1, mode1 | 0x80)  # RESTART
        _sleep_ms(1)

        # Totem pole output (OUTDRV=1) recommended for servo signals
        self.write8(_MODE2, 0x04)


def init():
    global _i2c, _pcas
    if _i2c is not None and _pcas:
        return

    if I2C is None or Pin is None:
        raise RuntimeError("machine.I2C/Pin not available (run on MicroPython hardware)")

    _i2c = I2C(
        int(config.I2C_ID),
        sda=Pin(int(config.I2C_SDA)),
        scl=Pin(int(config.I2C_SCL)),
        freq=int(getattr(config, "I2C_FREQ", 400_000)),
    )
    _sleep_ms(20)

    addrs = getattr(config, "PCA_ADDRS", [0x40])
    _pcas = [PCA9685(_i2c, addr) for addr in addrs]

    freq = getattr(config, "PCA_FREQ_HZ", 50)
    for p in _pcas:
        p.set_freq(freq)

    _sleep_ms(20)


def _angle_to_us(angle_deg):
    # Keep the classic 0..180 mapping unless user changes SERVO_US_MIN/MAX
    a = clamp(float(angle_deg), 0.0, 180.0)
    us_min = float(getattr(config, "SERVO_US_MIN", 500))
    us_max = float(getattr(config, "SERVO_US_MAX", 2500))
    return us_min + (us_max - us_min) * (a / 180.0)


def _us_to_ticks(us):
    freq = float(getattr(config, "PCA_FREQ_HZ", 50))
    period_us = 1_000_000.0 / freq
    ticks = int(round(float(us) * 4096.0 / period_us))
    return int(clamp(ticks, 0, 4095))


def _set_logical_channel(ch, angle_deg):
    init()
    ch = int(ch)

    mapping = getattr(config, "LOGICAL_TO_PCA", None)
    if mapping is None:
        raise RuntimeError("config_pca.LOGICAL_TO_PCA is missing")

    try:
        board_idx, board_ch = mapping[ch]
    except Exception:
        raise ValueError("Logical channel %d not in LOGICAL_TO_PCA" % ch)

    if int(board_idx) >= len(_pcas):
        raise ValueError(
            "Channel %d mapped to board %d, but only %d board(s) configured"
            % (ch, int(board_idx), len(_pcas))
        )

    us = _angle_to_us(angle_deg)
    ticks = _us_to_ticks(us)
    _pcas[int(board_idx)].set_pwm(int(board_ch), 0, ticks)


def set_servo(ch, angle_deg, t_ms=None):
    # t_ms accepted for compatibility, ignored (non-blocking)
    _set_logical_channel(int(ch), float(angle_deg))


def set_pose(pairs, t_ms=None):
    # t_ms accepted for compatibility, ignored (non-blocking)
    init()
    for ch, a in pairs:
        _set_logical_channel(int(ch), float(a))


def center_all(t_ms=None, channels=None):
    # t_ms accepted for compatibility, ignored (non-blocking)
    if channels is None:
        channels = range(int(getattr(config, "NUM_USED_SERVOS", 18)))
    set_pose([(ch, 90.0) for ch in channels], t_ms=None)