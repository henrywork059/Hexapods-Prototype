"""Virtual servo driver for simulation (no hardware I/O)."""

from __future__ import annotations

from . import config

_current_angle = [90.0] * len(config.SERVO_INVERT)


def init():
    return None


def set_servo(ch, angle_deg, t_ms=None):
    ch = int(ch)
    _current_angle[ch] = float(angle_deg)


def set_pose(pairs, t_ms=None):
    for ch, angle in pairs:
        set_servo(ch, angle, t_ms=t_ms)


def center_all(t_ms=240, channels=None):
    if channels is None:
        channels = range(len(_current_angle))
    set_pose([(ch, 90.0) for ch in channels], t_ms=t_ms)


def get_pose():
    return list(_current_angle)
