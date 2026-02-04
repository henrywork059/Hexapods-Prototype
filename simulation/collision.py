"""Collision and clearance checks for the simulation world."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

Vector = Tuple[float, float, float]


def v_sub(a: Vector, b: Vector) -> Vector:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_add(a: Vector, b: Vector) -> Vector:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_mul(a: Vector, s: float) -> Vector:
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a: Vector, b: Vector) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm2(a: Vector) -> float:
    return dot(a, a)


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def segment_segment_distance(p1: Vector, q1: Vector, p2: Vector, q2: Vector) -> float:
    """Shortest distance between two 3D line segments."""
    d1 = v_sub(q1, p1)
    d2 = v_sub(q2, p2)
    r = v_sub(p1, p2)

    a = dot(d1, d1)
    e = dot(d2, d2)
    f = dot(d2, r)

    if a <= 1e-12 and e <= 1e-12:
        return math.sqrt(norm2(v_sub(p1, p2)))
    if a <= 1e-12:
        t = clamp(f / e, 0.0, 1.0)
        closest = v_add(p2, v_mul(d2, t))
        return math.sqrt(norm2(v_sub(p1, closest)))
    if e <= 1e-12:
        s = clamp(-dot(d1, r) / a, 0.0, 1.0)
        closest = v_add(p1, v_mul(d1, s))
        return math.sqrt(norm2(v_sub(closest, p2)))

    b = dot(d1, d2)
    c = dot(d1, r)
    denom = a * e - b * b

    if abs(denom) > 1e-12:
        s = clamp((b * f - c * e) / denom, 0.0, 1.0)
    else:
        s = 0.0

    t = (b * s + f) / e
    if t < 0.0:
        t = 0.0
        s = clamp(-c / a, 0.0, 1.0)
    elif t > 1.0:
        t = 1.0
        s = clamp((b - c) / a, 0.0, 1.0)

    c1 = v_add(p1, v_mul(d1, s))
    c2 = v_add(p2, v_mul(d2, t))
    return math.sqrt(norm2(v_sub(c1, c2)))


def segment_ground_clearance(p1: Vector, p2: Vector, ground_z: float) -> float:
    """Return minimum z value relative to the ground plane."""
    return min(p1[2] - ground_z, p2[2] - ground_z)


def min_pairwise_distance(segments: Iterable[Tuple[Vector, Vector]]) -> float:
    segs = list(segments)
    if len(segs) < 2:
        return float("inf")
    min_d = float("inf")
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            d = segment_segment_distance(segs[i][0], segs[i][1], segs[j][0], segs[j][1])
            if d < min_d:
                min_d = d
    return min_d
