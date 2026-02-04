import sys

# ============================
# USER CONFIG (edit as needed)
# ============================
CMD_A = (20.0, 0.0, 0.0)
CMD_B = (-10.0, 5.0, 0.1)
DT_FRACTION = 0.25
# ============================

if ".." not in sys.path:
    sys.path.append("..")

import gait_tripod
from util import assert_equal, assert_true


def test_tripod_group_alternation_and_phase_timing():
    gait = gait_tripod.TripodGait()
    phase0 = gait.phase
    swing0, stance0 = gait._current_groups()

    gait.set_command(*CMD_A)
    gait.update(dt=gait.phase_T * DT_FRACTION, send=False)
    assert_equal(gait.phase, phase0, "Phase should not advance before phase_T")
    swing1, stance1 = gait._current_groups()
    assert_equal(swing0, swing1)
    assert_equal(stance0, stance1)

    gait.update(dt=gait.phase_T, send=False)
    assert_equal(gait.phase, 1 - phase0, "Phase should toggle after phase_T")
    swing2, stance2 = gait._current_groups()
    assert_true(swing2 != swing0, "Swing group should alternate")
    assert_true(stance2 != stance0, "Stance group should alternate")


def test_latched_command_updates_on_boundary():
    gait = gait_tripod.TripodGait()
    assert_equal(gait.cmd_latched, (0.0, 0.0, 0.0))

    gait.set_command(*CMD_B)
    gait.update(dt=gait.phase_T * DT_FRACTION, send=False)
    assert_equal(gait.cmd_latched, (0.0, 0.0, 0.0))

    gait.update(dt=gait.phase_T, send=False)
    assert_true(gait.cmd_latched != (0.0, 0.0, 0.0))
