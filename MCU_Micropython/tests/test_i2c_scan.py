import sys

# ============================
# USER CONFIG (edit as needed)
# ============================
I2C_ID = None
I2C_SDA = None
I2C_SCL = None
I2C_FREQ = None
PCA9685_ADDR = None
# ============================

if ".." not in sys.path:
    sys.path.append("..")

from util import RUNTIME_CONFIG, assert_true, require_hardware, skip


def _get_value(name, fallback):
    runtime = RUNTIME_CONFIG.get(name)
    if runtime is not None:
        return runtime
    if fallback is not None:
        return fallback
    try:
        import config
    except Exception:
        return None
    return getattr(config, name, None)


def test_i2c_scan():
    require_hardware()
    try:
        from machine import I2C, Pin
    except ImportError:
        skip("machine.I2C not available")

    i2c = I2C(
        int(_get_value("I2C_ID", I2C_ID) or 0),
        sda=Pin(int(_get_value("I2C_SDA", I2C_SDA) or 0)),
        scl=Pin(int(_get_value("I2C_SCL", I2C_SCL) or 1)),
        freq=int(_get_value("I2C_FREQ", I2C_FREQ) or 400_000),
    )
    found = i2c.scan()
    print("I2C devices:", [hex(addr) for addr in found])
    pca_addr = int(_get_value("PCA9685_ADDR", PCA9685_ADDR) or 0x40)
    assert_true(pca_addr in found, "PCA9685 address 0x%02X not found" % pca_addr)
