"""Bridge utilities for mapping IK outputs to MuJoCo actuator order."""

from __future__ import annotations

import math
from typing import Iterable, Sequence

JOINT_ORDER = [
    name
    for leg in range(6)
    for name in (f"leg{leg}_coxa", f"leg{leg}_femur", f"leg{leg}_tibia")
]

ACTUATOR_ORDER = [f"act_{name}" for name in JOINT_ORDER]


def mm_to_m(value: float) -> float:
    return float(value) * 0.001


def mm_vec_to_m(vec: Sequence[float]) -> tuple[float, float, float]:
    return (mm_to_m(vec[0]), mm_to_m(vec[1]), mm_to_m(vec[2]))


def _deg_to_rad(angle_deg: float) -> float:
    return float(angle_deg) * math.pi / 180.0


def flatten_ik_solutions(ik_solutions: Iterable[dict[str, float]]) -> list[float]:
    flat: list[float] = []
    for sol in ik_solutions:
        flat.extend([sol["coxa"], sol["femur"], sol["tibia"]])
    return flat


def map_ik_to_ctrl(
    ik_solutions: Iterable[dict[str, float]],
    degrees: bool = True,
) -> list[float]:
    targets = flatten_ik_solutions(ik_solutions)
    if degrees:
        return [_deg_to_rad(angle) for angle in targets]
    return [float(angle) for angle in targets]
