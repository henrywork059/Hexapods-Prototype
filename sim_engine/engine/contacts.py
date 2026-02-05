from __future__ import annotations

from dataclasses import dataclass

from .rigid_body import RigidBody, Vec2


@dataclass
class Contact:
    body_a: RigidBody
    body_b: RigidBody
    point: Vec2
    normal: Vec2
    penetration: float
