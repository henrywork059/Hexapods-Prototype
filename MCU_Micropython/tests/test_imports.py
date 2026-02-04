import sys

# ============================
# USER CONFIG (edit as needed)
# ============================
MODULES = [
    "config",
    "frames",
    "gait_tripod",
    "ik",
    "main",
    "servo_pca",
    "trajectory",
]
# ============================

if ".." not in sys.path:
    sys.path.append("..")

from util import assert_equal


def _ensure_config_alias():
    if "config_pca" in sys.modules:
        return
    try:
        import config as config_module
    except Exception:
        return
    sys.modules["config_pca"] = config_module


def test_imports():
    _ensure_config_alias()
    imported = []
    for name in MODULES:
        __import__(name)
        imported.append(name)
    assert_equal(len(imported), len(MODULES))
