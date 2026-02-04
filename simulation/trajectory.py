"""Foot trajectory generators (stance + swing) for simulation."""

from __future__ import annotations

import math


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def lerp(a, b, t):
    return a + (b - a) * t


def v_lerp(a, b, t):
    return (lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t))


def clamp01(t):
    return 0.0 if t < 0.0 else 1.0 if t > 1.0 else float(t)


def smoothstep3(t):
    t = clamp01(t)
    return t * t * (3.0 - 2.0 * t)


def smoothstep5(t):
    t = clamp01(t)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def ease(t, kind="quintic"):
    if kind == "linear":
        return clamp01(t)
    if kind == "cubic":
        return smoothstep3(t)
    return smoothstep5(t)


def stance(p0, p1, phase, ease_kind="linear"):
    s = ease(phase, ease_kind)
    return v_lerp(p0, p1, s)


def swing_parabolic(p0, p1, phase, h, ease_kind="quintic"):
    t = clamp01(phase)
    s = ease(t, ease_kind)
    p = v_lerp(p0, p1, s)
    lift = float(h) * 4.0 * t * (1.0 - t)
    return (p[0], p[1], p[2] + lift)


def swing_sine(p0, p1, phase, h, ease_kind="quintic"):
    t = clamp01(phase)
    s = ease(t, ease_kind)
    p = v_lerp(p0, p1, s)
    lift = float(h) * math.sin(math.pi * t)
    return (p[0], p[1], p[2] + lift)


def swing_bezier(p0, p1, phase, h, ease_kind="quintic", apex_xy=None):
    t = clamp01(phase)
    u = ease(t, ease_kind)

    if apex_xy is None:
        cx = 0.5 * (p0[0] + p1[0])
        cy = 0.5 * (p0[1] + p1[1])
    else:
        cx, cy = float(apex_xy[0]), float(apex_xy[1])

    cz = 0.5 * (p0[2] + p1[2]) + float(h)
    c = (cx, cy, cz)

    one = 1.0 - u
    return v_add(v_add(v_mul(p0, one * one), v_mul(c, 2.0 * one * u)), v_mul(p1, u * u))


def swing(p0, p1, phase, h, style="sine", ease_kind="quintic", apex_xy=None):
    if style == "parabolic":
        return swing_parabolic(p0, p1, phase, h, ease_kind=ease_kind)
    if style == "bezier":
        return swing_bezier(p0, p1, phase, h, ease_kind=ease_kind, apex_xy=apex_xy)
    return swing_sine(p0, p1, phase, h, ease_kind=ease_kind)


def sample_path(func, n=21):
    if n < 2:
        n = 2
    return [func(k / (n - 1)) for k in range(n)]
