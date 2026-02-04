"""
Shared test helpers for MicroPython-friendly tests.
"""

import sys

try:
    import time
except ImportError:
    time = None


RUNTIME_CONFIG = {
    "RUN_HARDWARE_TESTS": False,
    "HARDWARE_CONFIRM": False,
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
    if not RUNTIME_CONFIG.get("HARDWARE_CONFIRM", False):
        raise SkipTest(msg)


def require_servo_enable(msg="SAFE_SERVO_ENABLE is False"):
    if not RUNTIME_CONFIG.get("SAFE_SERVO_ENABLE", False):
        raise SkipTest(msg)


def _add_repo_paths():
    for path in ("..", "../.."):
        if path not in sys.path:
            sys.path.append(path)


def _clamp(value, lo, hi):
    return lo if value < lo else hi if value > hi else value


def _load_config():
    try:
        import config
    except Exception:
        return None
    return config


class ServoAdapter:
    def __init__(self, backend_name, backend, config):
        self.backend_name = backend_name
        self.backend = backend
        self.config = config
        self.controller = None
        if backend_name == "servo_controller":
            self.controller = backend.ServoController()

    def _clamp_deg(self, channel, deg):
        if self.config is None:
            return _clamp(float(deg), 0.0, 180.0)
        if hasattr(self.config, "SERVO_MIN_A") and hasattr(self.config, "SERVO_MAX_A"):
            lo = float(self.config.SERVO_MIN_A[int(channel)])
            hi = float(self.config.SERVO_MAX_A[int(channel)])
            return _clamp(float(deg), lo, hi)
        return _clamp(float(deg), 0.0, 180.0)

    def _deg_to_us(self, deg):
        if self.config is None:
            us_min = 500.0
            us_max = 2500.0
        else:
            us_min = float(getattr(self.config, "SERVO_US_MIN", 500))
            us_max = float(getattr(self.config, "SERVO_US_MAX", 2500))
        return us_min + (us_max - us_min) * (_clamp(float(deg), 0.0, 180.0) / 180.0)

    def _us_to_deg(self, us):
        if self.config is None:
            us_min = 500.0
            us_max = 2500.0
        else:
            us_min = float(getattr(self.config, "SERVO_US_MIN", 500))
            us_max = float(getattr(self.config, "SERVO_US_MAX", 2500))
        us = _clamp(float(us), us_min, us_max)
        return 180.0 * (us - us_min) / (us_max - us_min)

    def set_servo_deg(self, channel, deg, t_ms=None):
        deg = self._clamp_deg(channel, deg)
        if hasattr(self.backend, "set_servo"):
            return self.backend.set_servo(int(channel), float(deg), t_ms=t_ms)
        if hasattr(self.backend, "set_servo_deg"):
            return self.backend.set_servo_deg(int(channel), float(deg))
        if self.controller is not None:
            move_time = int(t_ms) if t_ms is not None else 240
            return self.controller.send_servo_command(int(channel), float(deg), move_time)
        raise RuntimeError("No servo angle output available for backend %s" % self.backend_name)

    def set_servo_us(self, channel, us, t_ms=None):
        if hasattr(self.backend, "set_servo_us"):
            return self.backend.set_servo_us(int(channel), int(us), t_ms=t_ms)
        return self.set_servo_deg(channel, self._us_to_deg(us), t_ms=t_ms)

    def neutral_all(self, t_ms=None):
        if hasattr(self.backend, "neutral_all"):
            return self.backend.neutral_all(t_ms=t_ms) if t_ms is not None else self.backend.neutral_all()
        if hasattr(self.backend, "center_all"):
            return self.backend.center_all(t_ms=t_ms) if t_ms is not None else self.backend.center_all()
        if self.controller is not None:
            move_time = int(t_ms) if t_ms is not None else 240
            return self.controller.neutral_position(move_time_ms=move_time)
        if hasattr(self.backend, "set_pose"):
            count = 18
            if self.config is not None:
                count = int(getattr(self.config, "NUM_USED_SERVOS", count))
            pairs = [(ch, 90.0) for ch in range(count)]
            return self.backend.set_pose(pairs, t_ms=t_ms)
        raise RuntimeError("No neutral_all implementation for backend %s" % self.backend_name)


def detect_servo_backend():
    _add_repo_paths()
    config = _load_config()

    for name in ("servo_out", "servo_controller"):
        try:
            mod = __import__(name)
        except Exception:
            mod = None
        if mod is not None:
            return ServoAdapter(name, mod, config)

    for name in ("servo_pca", "servo"):
        try:
            mod = __import__(name)
        except Exception:
            mod = None
        if mod is not None:
            return ServoAdapter(name, mod, config)

    raise SkipTest("No supported servo backend found")


def panic_stop(adapter, t_ms=300):
    return adapter.neutral_all(t_ms=t_ms)


def pretty_summary(passed, failed, skipped):
    total = passed + failed + skipped
    return "TOTAL=%d PASS=%d FAIL=%d SKIP=%d" % (total, passed, failed, skipped)
