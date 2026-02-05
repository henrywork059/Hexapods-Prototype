"""2D vector math utilities using millimeter units."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot


@dataclass(frozen=True)
class Vec2:
    """2D vector expressed in millimeters (mm)."""

    x: float
    y: float

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vec2":
        return Vec2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> "Vec2":
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> "Vec2":
        if scalar == 0:
            raise ValueError("Cannot divide by zero.")
        return Vec2(self.x / scalar, self.y / scalar)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: "Vec2") -> float:
        """2D cross product returning a scalar (z-component)."""
        return self.x * other.y - self.y * other.x

    def magnitude(self) -> float:
        return hypot(self.x, self.y)

    def normalize(self) -> "Vec2":
        mag = self.magnitude()
        if mag == 0:
            raise ValueError("Cannot normalize a zero-length vector.")
        return self / mag
