import sys

# ============================
# USER CONFIG (edit as needed)
# ============================
SERVO_CHANNEL = 0
NEUTRAL_DEG = 90.0
RAMP_TARGET_DEG = 92.0
RAMP_DURATION_MS = 800
STEP_DEG = 1.0
# ============================

if ".." not in sys.path:
    sys.path.append("..")

from util import (
    detect_servo_backend,
    panic_stop,
    require_hardware,
    require_servo_enable,
    sleep_ms,
    ticks_diff,
    ticks_ms,
)


def _maybe_set_freq(adapter):
    backend = adapter.backend
    if hasattr(backend, "init"):
        backend.init()
    config = adapter.config
    if config is None:
        return
    if hasattr(config, "PCA_FREQ_HZ"):
        config.PCA_FREQ_HZ = 50


def _ramp_deg(adapter, channel, start_deg, end_deg, duration_ms, step_deg):
    step = float(step_deg)
    if step <= 0:
        step = 1.0
    steps = max(1, int(abs(end_deg - start_deg) / step))
    delay = max(1, int(duration_ms / steps))
    t0 = ticks_ms()
    for i in range(steps + 1):
        frac = i / float(steps)
        angle = start_deg + (end_deg - start_deg) * frac
        adapter.set_servo_deg(channel, angle)
        sleep_ms(delay)
        if ticks_diff(ticks_ms(), t0) >= duration_ms:
            break


def test_pca9685_smoke():
    require_hardware()
    require_servo_enable()
    adapter = detect_servo_backend()
    _maybe_set_freq(adapter)
    try:
        adapter.set_servo_deg(SERVO_CHANNEL, NEUTRAL_DEG)
        _ramp_deg(adapter, SERVO_CHANNEL, NEUTRAL_DEG, RAMP_TARGET_DEG, RAMP_DURATION_MS, STEP_DEG)
    finally:
        panic_stop(adapter)
