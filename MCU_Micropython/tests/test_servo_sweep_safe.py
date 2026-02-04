import sys

if ".." not in sys.path:
    sys.path.append("..")

from util import require_hardware, require_servo_enable, sleep_ms, skip


def test_servo_sweep_safe():
    require_hardware()
    require_servo_enable()
    try:
        import servo_pca
    except ImportError:
        skip("servo_pca not available")

    servo_pca.set_servo(0, 90, t_ms=120)
    sleep_ms(200)
    servo_pca.set_servo(0, 85, t_ms=120)
    sleep_ms(200)
    servo_pca.set_servo(0, 95, t_ms=120)
    sleep_ms(200)
