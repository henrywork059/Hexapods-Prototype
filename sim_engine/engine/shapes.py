from __future__ import annotations

from dataclasses import dataclass


class Shape:
    def moment_of_inertia(self, mass: float) -> float:
        raise NotImplementedError


@dataclass
class Circle(Shape):
    radius: float

    def moment_of_inertia(self, mass: float) -> float:
        return 0.5 * mass * (self.radius ** 2)


@dataclass
class Box(Shape):
    half_width: float
    half_height: float

    def moment_of_inertia(self, mass: float) -> float:
        width = self.half_width * 2.0
        height = self.half_height * 2.0
        return (mass * (width ** 2 + height ** 2)) / 12.0
