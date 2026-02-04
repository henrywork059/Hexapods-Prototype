import sys

if ".." not in sys.path:
    sys.path.append("..")

from util import require_hardware, skip


def test_pca9685_smoke():
    require_hardware()
    try:
        import servo_pca
    except ImportError:
        skip("servo_pca not available")

    servo_pca.init()
