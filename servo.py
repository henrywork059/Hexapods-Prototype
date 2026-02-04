\
# servo.py
# Servo driver (PCA9685 I2C).
#
# Exports:
#   init()
#   set_servo(ch, angle_deg, t_ms=None)
#   set_pose([(ch, angle_deg), ...], t_ms=None)
#   center_all(t_ms=240, channels=range(18))
#
# Notes:
# - PCA9685 does NOT have built-in move timing. If t_ms is provided,
#   this module interpolates in software with small steps (blocking).
# - Angles are absolute servo angles in degrees (0..180 typical).

import time

# --- config import (supports either config.py or config_pca.py) ---
try:
    import config
except ImportError:
    import config_pca as config  # fallback if you didn't rename config_pca.py

try:
    from machine import I2C, Pin
except ImportError:
    I2C = None
    Pin = None

# PCA9685 registers
_MODE1      = 0x00
_MODE2      = 0x01
_PRESCALE   = 0xFE
_LED0_ON_L  = 0x06

_OSC_HZ = 25_000_000  # typical PCA9685 internal oscillator

_i2c = None
_pcas = []
_current_angle = [90.0] * int(getattr(config, "NUM_LOGICAL_CHANNELS", 24))


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
            int(on) & 0xFF, (int(on) >> 8) & 0xFF,
            int(off) & 0xFF, (int(off) >> 8) & 0xFF,
        ])
        self.i2c.writeto_mem(self.addr, base, data)

    def set_freq(self, freq_hz):
        # Standard sequence: sleep -> set prescale -> wake -> restart
        freq = float(freq_hz)
        prescale_val = int(round(_OSC_HZ / (4096.0 * freq) - 1.0))
        prescale_val = int(clamp(prescale_val, 3, 255))

        old_mode = self.read8(_MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10  # SLEEP bit
        self.write8(_MODE1, sleep_mode)
        _sleep_ms(1)

        self.write8(_PRESCALE, prescale_val)

        self.write8(_MODE1, old_mode)  # wake
        _sleep_ms(1)
        self.write8(_MODE1, old_mode | 0x80)  # RESTART
        _sleep_ms(1)

        # MODE2: totem pole (0x04) is common for servos
        self.write8(_MODE2, 0x04)


def init():
    """Initialize I2C + PCA board objects once."""
    global _i2c, _pcas
    if _i2c is not None and _pcas:
        return

    if I2C is None or Pin is None:
        raise RuntimeError("machine.I2C/Pin not available (run on MicroPython hardware).")

    _i2c = I2C(
        int(config.I2C_ID),
        sda=Pin(int(config.I2C_SDA)),
        scl=Pin(int(config.I2C_SCL)),
        freq=int(getattr(config, "I2C_FREQ", 400_000)),
    )
    _sleep_ms(20)

    _pcas = [PCA9685(_i2c, addr) for addr in getattr(config, "PCA_ADDRS", [0x40])]
    for p in _pcas:
        p.set_freq(getattr(config, "PCA_FREQ_HZ", 50))

    _sleep_ms(20)


def _angle_to_us(angle_deg):
    # Map 0..180 deg -> SERVO_US_MIN..SERVO_US_MAX
    a = clamp(float(angle_deg), 0.0, 180.0)
    us_min = float(getattr(config, "SERVO_US_MIN", 500))
    us_max = float(getattr(config, "SERVO_US_MAX", 2500))
    return us_min + (us_max - us_min) * (a / 180.0)


def _us_to_ticks(us):
    # ticks within 0..4095 for PCA, based on current PWM freq
    freq = float(getattr(config, "PCA_FREQ_HZ", 50))
    period_us = 1_000_000.0 / freq
    ticks = int(round(float(us) * 4096.0 / period_us))
    return int(clamp(ticks, 0, 4095))


def _set_logical_channel(ch, angle_deg):
    # Convert logical channel -> (board_index, board_channel) then write PWM
    init()

    ch = int(ch)
    board_idx, board_ch = getattr(config, "LOGICAL_TO_PCA")[ch]

    if board_idx >= len(_pcas):
        raise ValueError(
            "Logical channel %d mapped to board %d, but only %d board(s) configured"
            % (ch, board_idx, len(_pcas))
        )

    us = _angle_to_us(angle_deg)
    ticks = _us_to_ticks(us)
    _pcas[board_idx].set_pwm(int(board_ch), 0, ticks)


def set_servo(ch, angle_deg, t_ms=None):
    """Set one servo (logical channel) to angle_deg.

    If t_ms is provided, this interpolates from current to target (blocking).
    """
    ch = int(ch)
    target = float(angle_deg)
    if t_ms is None or int(t_ms) <= 0:
        _set_logical_channel(ch, target)
        _current_angle[ch] = target
        return

    steps = max(1, int(t_ms) // 20)  # ~50 Hz stepping
    start = float(_current_angle[ch])
    for k in range(1, steps + 1):
        a = start + (target - start) * (k / steps)
        _set_logical_channel(ch, a)
        _sleep_ms(int(t_ms) / steps)

    _current_angle[ch] = target


def set_pose(pairs, t_ms=None):
    """Set multiple servos in one pose.

    PCA can't truly sync, but we apply each step for all servos before waiting.
    If t_ms is provided, we do a software ramp for all channels together (blocking).
    """
    init()

    pairs = [(int(ch), float(a)) for (ch, a) in pairs]

    if t_ms is None or int(t_ms) <= 0:
        for ch, a in pairs:
            _set_logical_channel(ch, a)
            _current_angle[ch] = a
        return

    steps = max(1, int(t_ms) // 20)
    starts = {ch: float(_current_angle[ch]) for ch, _a in pairs}
    targets = {ch: float(a) for ch, a in pairs}

    for k in range(1, steps + 1):
        frac = k / steps
        for ch in targets:
            a = starts[ch] + (targets[ch] - starts[ch]) * frac
            _set_logical_channel(ch, a)
        _sleep_ms(int(t_ms) / steps)

    for ch in targets:
        _current_angle[ch] = targets[ch]


def center_all(t_ms=240, channels=None):
    """Move a set of channels to 90 deg."""
    if channels is None:
        channels = range(int(getattr(config, "NUM_USED_SERVOS", 18)))
    set_pose([(ch, 90.0) for ch in channels], t_ms=t_ms)
