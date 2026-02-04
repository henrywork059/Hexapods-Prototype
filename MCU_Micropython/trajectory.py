\
# trajectory.py
# Foot trajectory generators (stance + swing).
#
# Works in ANY frame you choose (hip-local is typical for IK).
# Conventions assumed by your project:
#   - +z is UP
#   - foot under hip is negative z
#
# You control "clearance" (lift height) via parameter h (mm).
#
# This module intentionally does NOT decide gait timing or which legs swing.
# It only provides smooth paths between two points.

import math

# -----------------------------
# Vector helpers
# -----------------------------
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


# -----------------------------
# Time shaping / easing
# -----------------------------
def clamp01(t):
    return 0.0 if t < 0.0 else 1.0 if t > 1.0 else float(t)

def smoothstep3(t):
    """Cubic smoothstep: 3t^2 - 2t^3."""
    t = clamp01(t)
    return t * t * (3.0 - 2.0 * t)

def smoothstep5(t):
    """Quintic smoothstep: 6t^5 - 15t^4 + 10t^3 (zero vel/acc at ends)."""
    t = clamp01(t)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)

def ease(t, kind="quintic"):
    if kind == "linear":
        return clamp01(t)
    if kind == "cubic":
        return smoothstep3(t)
    return smoothstep5(t)


# -----------------------------
# Stance trajectory
# -----------------------------
def stance(p0, p1, phase, ease_kind="linear"):
    """Stance path from p0 -> p1.

    Inputs:
      p0, p1: (x,y,z) tuples
      phase : 0..1 within stance
      ease_kind: 'linear' | 'cubic' | 'quintic'

    Returns:
      (x,y,z) at given phase.
    """
    s = ease(phase, ease_kind)
    return v_lerp(p0, p1, s)


# -----------------------------
# Swing trajectories
# -----------------------------
def swing_parabolic(p0, p1, phase, h, ease_kind="quintic"):
    """Swing with a smooth horizontal move + a lifted arc in z.

    z(phase) = lerp(z0, z1, s) + h * 4 * phase*(1-phase)

    The parabola term peaks at phase=0.5 with +h.
    """
    t = clamp01(phase)
    s = ease(t, ease_kind)

    # Horizontal/overall interpolation
    p = v_lerp(p0, p1, s)

    # Lift arc (always positive, peaks mid-swing)
    lift = float(h) * 4.0 * t * (1.0 - t)
    return (p[0], p[1], p[2] + lift)


def swing_sine(p0, p1, phase, h, ease_kind="quintic"):
    """Swing with sine lift (very smooth at ends).

    z(phase) = lerp(z0, z1, s) + h * sin(pi*phase)
    """
    t = clamp01(phase)
    s = ease(t, ease_kind)
    p = v_lerp(p0, p1, s)

    lift = float(h) * math.sin(math.pi * t)
    return (p[0], p[1], p[2] + lift)


def swing_bezier(p0, p1, phase, h, ease_kind="quintic", apex_xy=None):
    """Swing as a quadratic Bezier in 3D, with the control point lifted.

    If apex_xy is None:
      control point xy is midpoint of p0/p1
    If apex_xy is (x,y):
      control point xy is set to that (lets you arc around obstacles)
    Control point z is midpoint z + h.

    This is a good 'default' because it's simple and stable.
    """
    t = clamp01(phase)
    u = ease(t, ease_kind)

    if apex_xy is None:
        cx = 0.5 * (p0[0] + p1[0])
        cy = 0.5 * (p0[1] + p1[1])
    else:
        cx, cy = float(apex_xy[0]), float(apex_xy[1])

    cz = 0.5 * (p0[2] + p1[2]) + float(h)
    c = (cx, cy, cz)

    # Quadratic Bezier: B(u) = (1-u)^2 P0 + 2(1-u)u C + u^2 P1
    one = 1.0 - u
    return v_add(v_add(v_mul(p0, one * one), v_mul(c, 2.0 * one * u)), v_mul(p1, u * u))


def swing(p0, p1, phase, h, style="sine", ease_kind="quintic", apex_xy=None):
    """Unified swing API.

    style:
      - 'sine'      : smooth sine lift
      - 'parabolic' : simple parabola lift
      - 'bezier'    : quadratic Bezier (recommended flexible option)

    apex_xy only used for style='bezier'
    """
    if style == "parabolic":
        return swing_parabolic(p0, p1, phase, h, ease_kind=ease_kind)
    if style == "bezier":
        return swing_bezier(p0, p1, phase, h, ease_kind=ease_kind, apex_xy=apex_xy)
    return swing_sine(p0, p1, phase, h, ease_kind=ease_kind)


# -----------------------------
# Sampling helper (debug / plotting)
# -----------------------------
def sample_path(func, n=21):
    """Return n samples of func(phase) from 0..1 inclusive."""
    if n < 2:
        n = 2
    out = []
    for k in range(n):
        t = k / (n - 1)
        out.append(func(t))
    return out
