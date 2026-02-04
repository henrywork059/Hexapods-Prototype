from util import assert_true, ticks_diff, ticks_ms


def test_ticks_progression():
    t0 = ticks_ms()
    t1 = ticks_ms()
    diff = ticks_diff(t1, t0)
    assert_true(diff >= 0, "ticks should be monotonic (non-negative diff)")
