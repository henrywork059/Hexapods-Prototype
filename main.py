\
# main.py
# Hexapod main loop (I2C).
#
# Loads TripodGait and runs at DT.
# Optional interactive control over USB-serial:
#   w/s/a/d -> move, q/e -> yaw, x -> stop

import time

# --- config import (supports either config.py or config_pca.py) ---
try:
    import config
except ImportError:
    import config_pca as config

from gait_tripod import TripodGait

# ============================================================
# USER-EDIT SECTION
# ============================================================
INTERACTIVE = True
DEMO_SECONDS = 6.0

SPEED_V = 40.0      # mm/s
SPEED_WZ = 0.6      # rad/s

DT = float(getattr(config, "DT", 0.02))  # seconds


def sleep_s(dt_s):
    ms = int(max(0, dt_s * 1000))
    try:
        time.sleep_ms(ms)
    except AttributeError:
        time.sleep(dt_s)


def ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(a, b):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(a, b)
    return a - b


class CommandSource:
    """Non-blocking command reader from USB serial (stdin)."""
    def __init__(self):
        self.poll = None
        self.stdin = None
        self.ok = False

        try:
            import sys
            self.stdin = sys.stdin
        except ImportError:
            self.stdin = None

        try:
            import uselect
            self.poll = uselect.poll()
            self.poll.register(self.stdin, uselect.POLLIN)
            self.ok = True
        except Exception:
            self.poll = None
            self.ok = False

    def read_char(self):
        if not self.ok:
            return None
        try:
            ev = self.poll.poll(0)
            if not ev:
                return None
            return self.stdin.read(1)
        except Exception:
            return None


def command_from_key(ch, gait):
    if ch is None:
        return None
    ch = ch.strip()
    if not ch:
        return None

    if ch == "x":
        gait.set_command(0.0, 0.0, 0.0)
        return "STOP"

    if ch == "w":
        gait.set_command(+SPEED_V, 0.0, 0.0); return "FWD"
    if ch == "s":
        gait.set_command(-SPEED_V, 0.0, 0.0); return "BWD"
    if ch == "a":
        gait.set_command(0.0, +SPEED_V, 0.0); return "LEFT"
    if ch == "d":
        gait.set_command(0.0, -SPEED_V, 0.0); return "RIGHT"

    if ch == "q":
        gait.set_command(0.0, 0.0, +SPEED_WZ); return "YAW_L"
    if ch == "e":
        gait.set_command(0.0, 0.0, -SPEED_WZ); return "YAW_R"

    return None


def run():
    gait = TripodGait()
    try:
        gait.reset_pose(send=True)
    except Exception as e:
        print("Reset pose failed:", e)

    sleep_s(0.8)

    cmd_src = CommandSource() if INTERACTIVE else None
    interactive_ok = (cmd_src is not None and cmd_src.ok)

    if INTERACTIVE and not interactive_ok:
        print("Interactive input not available; running demo mode.")
    if interactive_ok:
        print("Interactive ready: w/s/a/d, q/e, x")

    gait.set_command(0.0, 0.0, 0.0)

    last = ticks_ms()
    demo_t0 = last

    while True:
        now = ticks_ms()
        dt_ms = ticks_diff(now, last)
        last = now

        dt = dt_ms / 1000.0
        if dt < 0.0 or dt > 0.2:
            dt = DT

        if interactive_ok:
            ch = cmd_src.read_char()
            label = command_from_key(ch, gait)
            if label:
                print("CMD:", label)
        else:
            t_demo = ticks_diff(now, demo_t0) / 1000.0
            if t_demo < DEMO_SECONDS:
                gait.set_command(+SPEED_V, 0.0, 0.0)
            else:
                gait.set_command(0.0, 0.0, 0.0)
                gait.update(dt=DT, send=True)
                print("Demo finished.")
                break

        gait.update(dt=dt, send=True)
        sleep_s(max(0.0, DT - dt))


if __name__ == "__main__":
    run()
