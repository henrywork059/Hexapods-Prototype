from util import assert_true, sleep_ms, ticks_diff, ticks_ms

# ============================
# USER CONFIG (edit as needed)
# ============================
SAMPLE_COUNT = 5
SLEEP_MS = 2
# ============================


def test_ticks_progression():
    t_prev = ticks_ms()
    for _ in range(SAMPLE_COUNT):
        sleep_ms(SLEEP_MS)
        t_now = ticks_ms()
        diff = ticks_diff(t_now, t_prev)
        assert_true(diff >= 0, "ticks should be monotonic (non-negative diff)")
        t_prev = t_now
