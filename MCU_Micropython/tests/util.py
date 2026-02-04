"""
Shared test helpers for MicroPython-friendly tests.
"""

try:
    import time
except ImportError:
    time = None


RUNTIME_CONFIG = {
    "RUN_HARDWARE_TESTS": False,
    "SAFE_SERVO_ENABLE": False,
    "I2C_ID": 0,
    "I2C_SDA": 0,
    "I2C_SCL": 1,
    "I2C_FREQ": 400_000,
    "PCA9685_ADDR": 0x40,
}


class SkipTest(Exception):
    """Raised to mark a test as skipped."""


def set_runtime_config(config):
    for key, value in config.items():
        RUNTIME_CONFIG[key] = value


def ticks_ms():
    if time is None:
        return 0
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(new, old):
    if time is None:
        return 0
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new, old)
    return int(new - old)


def sleep_ms(ms):
    if time is None:
        return
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(int(ms))
    else:
        time.sleep(float(ms) / 1000.0)


def assert_true(value, msg=None):
    if not value:
        raise AssertionError(msg or "Expected value to be truthy")


def assert_equal(a, b, msg=None):
    if a != b:
        raise AssertionError(msg or "Expected %r == %r" % (a, b))


def assert_almost_equal(a, b, tol=1e-6, msg=None):
    if abs(a - b) > tol:
        raise AssertionError(msg or "Expected %r ~= %r (tol=%s)" % (a, b, tol))


def skip(msg="skipped"):
    raise SkipTest(msg)


def require_hardware(msg="Hardware tests disabled"):
    if not RUNTIME_CONFIG.get("RUN_HARDWARE_TESTS", False):
        raise SkipTest(msg)


def require_servo_enable(msg="SAFE_SERVO_ENABLE is False"):
    if not RUNTIME_CONFIG.get("SAFE_SERVO_ENABLE", False):
        raise SkipTest(msg)


def pretty_summary(passed, failed, skipped):
    total = passed + failed + skipped
    return "TOTAL=%d PASS=%d FAIL=%d SKIP=%d" % (total, passed, failed, skipped)
