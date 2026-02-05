from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from .materials import Material
from .shapes import Shape


@dataclass
class Vec2:
    x: float
    z: float

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.z + other.z)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec2":
        return Vec2(self.x * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> "Vec2":
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> "Vec2":
        return Vec2(self.x / scalar, self.z / scalar)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.z * other.z

    def length(self) -> float:
        return (self.x * self.x + self.z * self.z) ** 0.5

    def normalized(self) -> "Vec2":
        length = self.length()
        if length == 0.0:
            return Vec2(0.0, 0.0)
        return self / length


def perp(v: Vec2) -> Vec2:
    return Vec2(-v.z, v.x)


def cross_scalar_vec(scalar: float, v: Vec2) -> Vec2:
    return Vec2(-scalar * v.z, scalar * v.x)


def cross_vec_vec(a: Vec2, b: Vec2) -> float:
    return a.x * b.z - a.z * b.x


class BodyType(Enum):
    STATIC = "static"
    DYNAMIC = "dynamic"
    KINEMATIC = "kinematic"


@dataclass
class RigidBody:
    shape: Shape
    body_type: BodyType = BodyType.DYNAMIC
    position: Vec2 = field(default_factory=lambda: Vec2(0.0, 0.0))
    angle: float = 0.0
    velocity: Vec2 = field(default_factory=lambda: Vec2(0.0, 0.0))
    angular_velocity: float = 0.0
    force: Vec2 = field(default_factory=lambda: Vec2(0.0, 0.0))
    torque: float = 0.0
    mass: float = 1.0
    material: Optional[Material] = None

    def __post_init__(self) -> None:
        if self.material is None:
            self.material = Material()
        if self.body_type == BodyType.DYNAMIC:
            self.inv_mass = 0.0 if self.mass == 0.0 else 1.0 / self.mass
            inertia = self.shape.moment_of_inertia(self.mass)
            self.inv_inertia = 0.0 if inertia == 0.0 else 1.0 / inertia
        else:
            self.inv_mass = 0.0
            self.inv_inertia = 0.0

    def apply_force(self, force: Vec2) -> None:
        if self.body_type != BodyType.DYNAMIC:
            return
        self.force = self.force + force

    def apply_impulse(self, impulse: Vec2, contact_vector: Vec2) -> None:
        if self.body_type != BodyType.DYNAMIC:
            return
        self.velocity = self.velocity + impulse * self.inv_mass
        self.angular_velocity += cross_vec_vec(contact_vector, impulse) * self.inv_inertia

    def clear_forces(self) -> None:
        self.force = Vec2(0.0, 0.0)
        self.torque = 0.0
