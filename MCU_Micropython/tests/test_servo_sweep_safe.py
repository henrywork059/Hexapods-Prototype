import sys

# ============================
# USER CONFIG (edit as needed)
# ============================
SERVO_CHANNEL = 0
SWEEP_ALL_CHANNELS = False
NEUTRAL_DEG = 90.0
DELTA_DEG = 10.0
RAMP_DURATION_MS = 800
STEP_DEG = 1.0
# ============================

if ".." not in sys.path:
    sys.path.append("..")

from util import detect_servo_backend, panic_stop, require_hardware, require_servo_enable, sleep_ms


def _ramp_deg(adapter, channel, start_deg, end_deg, duration_ms, step_deg):
    step = float(step_deg)
    if step <= 0:
        step = 1.0
    steps = max(1, int(abs(end_deg - start_deg) / step))
    delay = max(1, int(duration_ms / steps))
    for i in range(steps + 1):
        frac = i / float(steps)
        angle = start_deg + (end_deg - start_deg) * frac
        adapter.set_servo_deg(channel, angle)
        sleep_ms(delay)


def _channels(adapter):
    if SWEEP_ALL_CHANNELS and adapter.config is not None:
        return list(range(int(getattr(adapter.config, "NUM_USED_SERVOS", 18))))
    return [SERVO_CHANNEL]


def test_servo_sweep_safe():
    require_hardware()
    require_servo_enable()
    adapter = detect_servo_backend()
    channels = _channels(adapter)
    try:
        for ch in channels:
            adapter.set_servo_deg(ch, NEUTRAL_DEG)
            _ramp_deg(adapter, ch, NEUTRAL_DEG, NEUTRAL_DEG - DELTA_DEG, RAMP_DURATION_MS, STEP_DEG)
            _ramp_deg(adapter, ch, NEUTRAL_DEG - DELTA_DEG, NEUTRAL_DEG + DELTA_DEG, RAMP_DURATION_MS, STEP_DEG)
            _ramp_deg(adapter, ch, NEUTRAL_DEG + DELTA_DEG, NEUTRAL_DEG, RAMP_DURATION_MS, STEP_DEG)
    finally:
        panic_stop(adapter)
