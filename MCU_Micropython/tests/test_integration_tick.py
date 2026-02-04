import sys

# ============================
# USER CONFIG (edit as needed)
# ============================
N_TICKS = 5
DT = 0.02
COMMAND = (10.0, 0.0, 0.0)
# ============================

if ".." not in sys.path:
    sys.path.append("..")

import gait_tripod
import ik
from util import assert_equal, assert_true


def test_integration_tick():
    calls = []
    original = ik.move_all_legs_xyz

    def _stub_move_all_legs_xyz(targets, t_ms=None):
        calls.append((targets, t_ms))
        return []

    ik.move_all_legs_xyz = _stub_move_all_legs_xyz
    try:
        gait = gait_tripod.TripodGait()
        gait.set_command(*COMMAND)
        for _ in range(N_TICKS):
            pose = gait.update(dt=DT, send=True)
            assert_equal(len(pose), 6)
        assert_true(len(calls) >= N_TICKS, "Expected servo output per tick")
    finally:
        ik.move_all_legs_xyz = original
