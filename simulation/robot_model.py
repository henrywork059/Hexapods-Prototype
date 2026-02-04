"""Kinematic robot model for simulation (no physics)."""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Tuple

from . import config
from . import frames

Vector = Tuple[float, float, float]


def _rot_z(theta: float, p: Vector) -> Vector:
    c = math.cos(theta)
    s = math.sin(theta)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1], p[2])


def leg_points_hip(coxa_deg: float, femur_deg: float, tibia_deg: float) -> List[Vector]:
    """Return hip->coxa->femur->tibia points in hip-local frame."""
    theta_c = math.radians(coxa_deg)
    theta_f = math.radians(femur_deg)
    theta_t = math.radians(tibia_deg)

    coxa_end = (config.COXA_L, 0.0, 0.0)
    femur_end = (
        config.COXA_L + config.FEMUR_L * math.cos(theta_f),
        0.0,
        config.FEMUR_L * math.sin(theta_f),
    )
    tibia_end = (
        config.COXA_L + config.FEMUR_L * math.cos(theta_f) + config.TIBIA_L * math.cos(theta_f + theta_t),
        0.0,
        config.FEMUR_L * math.sin(theta_f) + config.TIBIA_L * math.sin(theta_f + theta_t),
    )

    return [
        (0.0, 0.0, 0.0),
        _rot_z(theta_c, coxa_end),
        _rot_z(theta_c, femur_end),
        _rot_z(theta_c, tibia_end),
    ]


def leg_segments_body(leg_idx: int, angles: Dict[str, float]) -> List[Tuple[Vector, Vector]]:
    points_H = leg_points_hip(angles["coxa"], angles["femur"], angles["tibia"])
    points_B = [frames.hip_point_to_body(leg_idx, p) for p in points_H]
    return list(zip(points_B[:-1], points_B[1:]))


class RobotModel:
    """Stores the current joint angles and provides segment geometry."""

    def __init__(self):
        self.joint_angles: List[Dict[str, float]] = [
            {"coxa": 0.0, "femur": 0.0, "tibia": 0.0} for _ in range(6)
        ]

    def update_from_solutions(self, sols: Iterable[Dict[str, float]]):
        for idx, sol in enumerate(sols):
            if idx < len(self.joint_angles):
                self.joint_angles[idx] = {
                    "coxa": float(sol["coxa"]),
                    "femur": float(sol["femur"]),
                    "tibia": float(sol["tibia"]),
                }

    def all_segments_body(self) -> List[Tuple[Vector, Vector]]:
        segments: List[Tuple[Vector, Vector]] = []
        for leg_idx, angles in enumerate(self.joint_angles):
            segments.extend(leg_segments_body(leg_idx, angles))
        return segments

    def segments_by_leg(self) -> List[List[Tuple[Vector, Vector]]]:
        return [leg_segments_body(leg_idx, angles) for leg_idx, angles in enumerate(self.joint_angles)]

    def foot_positions_body(self) -> List[Vector]:
        feet: List[Vector] = []
        for leg_idx, angles in enumerate(self.joint_angles):
            points_H = leg_points_hip(angles["coxa"], angles["femur"], angles["tibia"])
            foot_B = frames.hip_point_to_body(leg_idx, points_H[-1])
            feet.append(foot_B)
        return feet
