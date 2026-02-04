\
# main.py
# Hexapod main loop (Tripod gait, latched commands).
#
# This is a minimal, bring-up friendly entry point:
#   - loads config
#   - creates TripodGait
#   - resets to neutral
#   - runs a loop at DT
#
# Optional interactive control over USB-serial:
#   Type commands like:
#     w  -> forward
#     s  -> backward
#     a  -> left
#     d  -> right
#     q  -> yaw left
#     e  -> yaw right
#     x  -> stop (zero command)
#
# If interactive input isn't available, it will run a short demo and stop.

import time

# --- config import (supports either config.py or config_pca.py) ---
try:
    import config
except ImportError:
    import config_pca as config  # fallback if you didn't rename config_pca.py

from gait_tripod import TripodGait


# ============================================================
# USER-EDIT SECTION
# ============================================================

# Set to True if you want keyboard-like control from USB serial.
INTERACTIVE = True

# Demo fallback (used when interactive input isn't available)
DEMO_SECONDS = 6.0

# Default walking speeds (units: mm/s, rad/s)
SPEED_V = 40.0      # translation speed
SPEED_WZ = 0.6      # yaw speed

# Loop timing
DT = float(getattr(config, "DT", 0.02))   # seconds


# ============================================================
# Helpers
# ============================================================

def sleep_s(dt_s):
    """Sleep for dt_s seconds (works in MicroPython and CPython)."""
    ms = int(max(0, dt_s * 1000))
    try:
        time.sleep_ms(ms)
    except AttributeError:
        time.sleep(dt_s)


def ticks_ms():
    """Monotonic ms timer (MicroPython + CPython)."""
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_diff(a, b):
    """Difference in ms for MicroPython ticks or plain ints."""
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(a, b)
    return a - b


class CommandSource:
    """Non-blocking command reader from USB serial (stdin).
    Uses uselect.poll if available.
    """
    def __init__(self):
        self.poll = None
        self.stdin = None
        self.ok = False

        try:
            import sys
            self.stdin = sys.stdin
        except ImportError:
            self.stdin = None

        # MicroPython usually has uselect.poll()
        try:
            import uselect
            self.poll = uselect.poll()
            self.poll.register(self.stdin, uselect.POLLIN)
            self.ok = True
        except Exception:
            self.poll = None
            self.ok = False

    def read_char(self):
        """Return a single char if available, else None."""
        if not self.ok:
            return None
        try:
            ev = self.poll.poll(0)
            if not ev:
                return None
            ch = self.stdin.read(1)
            return ch
        except Exception:
            return None


def command_from_key(ch, gait):
    """Map 1-key command to (vx, vy, wz)."""
    if ch is None:
        return None

    ch = ch.strip()
    if not ch:
        return None

    # Stop
    if ch == "x":
        gait.set_command(0.0, 0.0, 0.0)
        return "STOP"

    # Translation
    if ch == "w":
        gait.set_command(+SPEED_V, 0.0, 0.0)
        return "FWD"
    if ch == "s":
        gait.set_command(-SPEED_V, 0.0, 0.0)
        return "BWD"
    if ch == "a":
        gait.set_command(0.0, +SPEED_V, 0.0)
        return "LEFT"
    if ch == "d":
        gait.set_command(0.0, -SPEED_V, 0.0)
        return "RIGHT"

    # Yaw
    if ch == "q":
        gait.set_command(0.0, 0.0, +SPEED_WZ)
        return "YAW_L"
    if ch == "e":
        gait.set_command(0.0, 0.0, -SPEED_WZ)
        return "YAW_R"

    return None


# ============================================================
# Main
# ============================================================

def run():
    gait = TripodGait()

    # Move to neutral pose (blocking)
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
        print("Interactive mode ready.")
        print("Keys: w/s/a/d (move), q/e (yaw), x (stop).")

    # Default command
    gait.set_command(0.0, 0.0, 0.0)

    # Timing loop
    last = ticks_ms()
    demo_t0 = last

    while True:
        now = ticks_ms()
        dt_ms = ticks_diff(now, last)
        last = now

        # Clamp dt to avoid crazy jumps
        dt = dt_ms / 1000.0
        if dt < 0.0:
            dt = DT
        if dt > 0.2:
            dt = DT

        if interactive_ok:
            ch = cmd_src.read_char()
            label = command_from_key(ch, gait)
            if label:
                print("CMD:", label)
        else:
            # Demo: walk forward for DEMO_SECONDS then stop and exit
            t_demo = ticks_diff(now, demo_t0) / 1000.0
            if t_demo < DEMO_SECONDS:
                gait.set_command(+SPEED_V, 0.0, 0.0)
            else:
                gait.set_command(0.0, 0.0, 0.0)
                gait.update(dt=DT, send=True)
                print("Demo finished.")
                break

        gait.update(dt=dt, send=True)

        # Maintain loop rate (best effort)
        sleep_s(max(0.0, DT - dt))


if __name__ == "__main__":
    run()
