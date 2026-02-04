"""3D simulation world container (no physics)."""

from __future__ import annotations

from typing import Dict, List, Tuple

from . import frames
from . import collision
from .robot_model import RobotModel

Vector = Tuple[float, float, float]


def _to_world(points_B: List[Vector], body_pos: Vector, body_rpy: Vector) -> List[Vector]:
    return [frames.body_point_to_world(p, body_pos, body_rpy) for p in points_B]


class World:
    def __init__(self, ground_z: float = 0.0, body_pos=(0.0, 0.0, 80.0), body_rpy=(0.0, 0.0, 0.0)):
        self.ground_z = float(ground_z)
        self.body_pos = tuple(float(v) for v in body_pos)
        self.body_rpy = tuple(float(v) for v in body_rpy)

    def evaluate(self, robot: RobotModel) -> Dict[str, float]:
        segments_by_leg = robot.segments_by_leg()

        world_segments_by_leg: List[List[Tuple[Vector, Vector]]] = []
        min_ground = float("inf")
        ground_hits = 0

        for leg_segments in segments_by_leg:
            leg_world: List[Tuple[Vector, Vector]] = []
            for seg_idx, (p_B, q_B) in enumerate(leg_segments):
                p_W, q_W = _to_world([p_B, q_B], self.body_pos, self.body_rpy)
                leg_world.append((p_W, q_W))

                clearance = collision.segment_ground_clearance(p_W, q_W, self.ground_z)
                min_ground = min(min_ground, clearance)

                is_foot_segment = seg_idx == len(leg_segments) - 1
                if not is_foot_segment and clearance < 0.0:
                    ground_hits += 1
            world_segments_by_leg.append(leg_world)

        min_link_distance = float("inf")
        for i, segs_a in enumerate(world_segments_by_leg):
            for j in range(i + 1, len(world_segments_by_leg)):
                segs_b = world_segments_by_leg[j]
                for seg_a in segs_a:
                    for seg_b in segs_b:
                        d = collision.segment_segment_distance(seg_a[0], seg_a[1], seg_b[0], seg_b[1])
                        if d < min_link_distance:
                            min_link_distance = d

        return {
            "min_ground_clearance": min_ground,
            "ground_penetrations": float(ground_hits),
            "min_link_distance": min_link_distance,
        }
