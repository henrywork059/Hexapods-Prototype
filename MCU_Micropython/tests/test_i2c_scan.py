import sys

if ".." not in sys.path:
    sys.path.append("..")

from util import require_hardware, skip


def test_i2c_scan():
    require_hardware()
    try:
        from machine import I2C, Pin
    except ImportError:
        skip("machine.I2C not available")

    import config

    i2c = I2C(
        int(getattr(config, "I2C_ID", 0)),
        sda=Pin(int(getattr(config, "I2C_SDA", 0))),
        scl=Pin(int(getattr(config, "I2C_SCL", 1))),
        freq=int(getattr(config, "I2C_FREQ", 400_000)),
    )
    _ = i2c.scan()
