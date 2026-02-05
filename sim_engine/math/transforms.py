"""Geometry transforms in millimeter units (mm) without hidden scaling."""

from __future__ import annotations

from math import cos, sin

from .vec2 import Vec2


def rotate_vec2(vec: Vec2, angle_rad: float) -> Vec2:
    """Rotate a vector about the origin by angle_rad (radians)."""
    cos_a = cos(angle_rad)
    sin_a = sin(angle_rad)
    return Vec2(
        vec.x * cos_a - vec.y * sin_a,
        vec.x * sin_a + vec.y * cos_a,
    )


def rotate_point(point: Vec2, origin: Vec2, angle_rad: float) -> Vec2:
    """Rotate a point around an origin by angle_rad (radians)."""
    return origin + rotate_vec2(point - origin, angle_rad)


def box_corners(center: Vec2, size: Vec2, angle_rad: float) -> tuple[Vec2, Vec2, Vec2, Vec2]:
    """Return the four corners of an axis-aligned box after rotation.

    Args:
        center: Box center in millimeters.
        size: Box size (width, height) in millimeters.
        angle_rad: Rotation in radians.
    """
    half = Vec2(size.x * 0.5, size.y * 0.5)
    local_corners = (
        Vec2(-half.x, -half.y),
        Vec2(half.x, -half.y),
        Vec2(half.x, half.y),
        Vec2(-half.x, half.y),
    )
    return tuple(center + rotate_vec2(corner, angle_rad) for corner in local_corners)


def transform_box_points(
    center: Vec2,
    size: Vec2,
    angle_rad: float,
    points: tuple[Vec2, ...],
) -> tuple[Vec2, ...]:
    """Transform local box points into world space using mm units.

    Points are interpreted relative to the box center in millimeters.
    """
    return tuple(center + rotate_vec2(point, angle_rad) for point in points)
