"""
Run all tests in this folder.
"""

# ============================
# USER CONFIG (edit as needed)
# ============================
RUN_HARDWARE_TESTS = False
SAFE_SERVO_ENABLE = False

# Default I2C settings (override to match your board)
I2C_ID = 0
I2C_SDA = 0
I2C_SCL = 1
I2C_FREQ = 400_000

# Default PCA9685 address
PCA9685_ADDR = 0x40
# ============================

import sys

if ".." not in sys.path:
    sys.path.append("..")

from util import SkipTest, pretty_summary, set_runtime_config


def _confirm_hardware():
    if not RUN_HARDWARE_TESTS:
        return False
    try:
        resp = input("Type YES to move servos and run hardware tests: ")
    except Exception:
        resp = ""
    if resp.strip() != "YES":
        print("Hardware tests disabled (confirmation failed).")
        return False
    return True


def _load_modules():
    module_names = [
        "test_imports",
        "test_config_sanity",
        "test_timebase",
        "test_i2c_scan",
        "test_pca9685_smoke",
        "test_servo_sweep_safe",
        "test_ik_sanity",
        "test_trajectory_math",
        "test_gait_state_machine",
        "test_integration_tick",
    ]
    modules = []
    import_errors = []
    for name in module_names:
        try:
            modules.append(__import__(name))
        except Exception as exc:
            import_errors.append((name, exc))
    return modules, import_errors


def _collect_tests(modules):
    tests = []
    for mod in modules:
        for name in dir(mod):
            if name.startswith("test_"):
                func = getattr(mod, name)
                if callable(func):
                    tests.append(("%s.%s" % (mod.__name__, name), func))
    return tests


def _run_minimal(tests, import_errors):
    passed = 0
    failed = 0
    skipped = 0

    for name, exc in import_errors:
        failed += 1
        print("FAIL import %s: %r" % (name, exc))

    for name, func in tests:
        try:
            func()
        except SkipTest as exc:
            skipped += 1
            print("SKIP %s: %s" % (name, exc))
        except Exception as exc:
            failed += 1
            print("FAIL %s: %r" % (name, exc))
        else:
            passed += 1
            print("PASS %s" % name)

    print(pretty_summary(passed, failed, skipped))
    return failed == 0


def _run_with_unittest(tests, import_errors, unittest_module):
    if import_errors:
        for name, exc in import_errors:
            print("FAIL import %s: %r" % (name, exc))
        print(pretty_summary(0, len(import_errors), 0))
        return False

    class FunctionTests(unittest_module.TestCase):
        pass

    def _wrap(func):
        def _inner(self):
            try:
                func()
            except SkipTest as exc:
                if hasattr(unittest_module, "SkipTest"):
                    raise unittest_module.SkipTest(str(exc))
                raise
        return _inner

    for name, func in tests:
        setattr(FunctionTests, name.replace(".", "_"), _wrap(func))

    suite = unittest_module.TestSuite()
    for name in dir(FunctionTests):
        if name.startswith("test_"):
            suite.addTest(FunctionTests(name))

    runner = getattr(unittest_module, "TextTestRunner", None)
    if runner is None:
        return _run_minimal(tests, import_errors)

    result = runner().run(suite)
    failures = len(getattr(result, "failures", [])) + len(getattr(result, "errors", []))
    skipped = len(getattr(result, "skipped", [])) if hasattr(result, "skipped") else 0
    passed = max(0, result.testsRun - failures - skipped)
    print(pretty_summary(passed, failures, skipped))
    return failures == 0


def main():
    hardware_ok = _confirm_hardware()
    set_runtime_config(
        {
            "RUN_HARDWARE_TESTS": bool(hardware_ok),
            "SAFE_SERVO_ENABLE": bool(SAFE_SERVO_ENABLE),
            "I2C_ID": int(I2C_ID),
            "I2C_SDA": int(I2C_SDA),
            "I2C_SCL": int(I2C_SCL),
            "I2C_FREQ": int(I2C_FREQ),
            "PCA9685_ADDR": int(PCA9685_ADDR),
            "HARDWARE_CONFIRM": bool(hardware_ok),
        }
    )

    modules, import_errors = _load_modules()
    tests = _collect_tests(modules)

    try:
        import unittest as unittest_module
    except Exception:
        unittest_module = None

    if unittest_module is None:
        ok = _run_minimal(tests, import_errors)
    else:
        ok = _run_with_unittest(tests, import_errors, unittest_module)

    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
