from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Material:
    friction: float = 0.6
    restitution: float = 0.0


def combine_friction(a: Material, b: Material) -> float:
    return (a.friction * b.friction) ** 0.5


def combine_restitution(a: Material, b: Material) -> float:
    return max(a.restitution, b.restitution)
