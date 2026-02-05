"""Replay recording and playback helpers."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from sim_engine.engine.collision import detect_contacts
from sim_engine.engine.rigid_body import BodyType, RigidBody, Vec2
from sim_engine.engine.shapes import Box, Circle, Shape


def _serialize_vec2(value: Vec2) -> dict[str, float]:
    return {"x": value.x, "z": value.z}


def _deserialize_vec2(payload: dict[str, Any]) -> Vec2:
    return Vec2(float(payload["x"]), float(payload["z"]))


def _serialize_shape(shape: Shape) -> dict[str, Any]:
    if isinstance(shape, Box):
        return {"type": "box", "half_width": shape.half_width, "half_height": shape.half_height}
    if isinstance(shape, Circle):
        return {"type": "circle", "radius": shape.radius}
    raise ValueError(f"Unsupported shape for replay: {shape}")


def _deserialize_shape(payload: dict[str, Any]) -> Shape:
    shape_type = payload.get("type")
    if shape_type == "box":
        return Box(float(payload["half_width"]), float(payload["half_height"]))
    if shape_type == "circle":
        return Circle(float(payload["radius"]))
    raise ValueError(f"Unsupported shape type for replay: {shape_type}")


@dataclass
class BodySnapshot:
    """Serialized snapshot of a rigid body."""

    shape: dict[str, Any]
    body_type: str
    mass: float
    position: dict[str, float]
    angle: float
    velocity: dict[str, float]
    angular_velocity: float

    @classmethod
    def from_body(cls, body: RigidBody) -> "BodySnapshot":
        return cls(
            shape=_serialize_shape(body.shape),
            body_type=body.body_type.value,
            mass=body.mass,
            position=_serialize_vec2(body.position),
            angle=body.angle,
            velocity=_serialize_vec2(body.velocity),
            angular_velocity=body.angular_velocity,
        )

    def to_body(self) -> RigidBody:
        return RigidBody(
            shape=_deserialize_shape(self.shape),
            body_type=BodyType(self.body_type),
            position=_deserialize_vec2(self.position),
            angle=self.angle,
            velocity=_deserialize_vec2(self.velocity),
            angular_velocity=self.angular_velocity,
            mass=self.mass,
        )

    def to_json(self) -> dict[str, Any]:
        return {
            "shape": self.shape,
            "body_type": self.body_type,
            "mass": self.mass,
            "position": self.position,
            "angle": self.angle,
            "velocity": self.velocity,
            "angular_velocity": self.angular_velocity,
        }

    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> "BodySnapshot":
        return cls(
            shape=payload["shape"],
            body_type=str(payload["body_type"]),
            mass=float(payload.get("mass", 1.0)),
            position=payload["position"],
            angle=float(payload.get("angle", 0.0)),
            velocity=payload["velocity"],
            angular_velocity=float(payload.get("angular_velocity", 0.0)),
        )


@dataclass
class FrameSnapshot:
    """State for a single simulation step."""

    time: float
    contact_count: int
    bodies: list[BodySnapshot] = field(default_factory=list)

    @classmethod
    def from_world(cls, bodies: Iterable[RigidBody], time: float, contact_count: int) -> "FrameSnapshot":
        return cls(
            time=time,
            contact_count=contact_count,
            bodies=[BodySnapshot.from_body(body) for body in bodies],
        )

    def to_json(self) -> dict[str, Any]:
        return {
            "time": self.time,
            "contact_count": self.contact_count,
            "bodies": [body.to_json() for body in self.bodies],
        }

    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> "FrameSnapshot":
        return cls(
            time=float(payload.get("time", 0.0)),
            contact_count=int(payload.get("contact_count", 0)),
            bodies=[BodySnapshot.from_json(body) for body in payload.get("bodies", [])],
        )


@dataclass
class ReplayRecording:
    """Collection of recorded frames and metadata."""

    frames: list[FrameSnapshot] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> dict[str, Any]:
        return {
            "metadata": self.metadata,
            "frames": [frame.to_json() for frame in self.frames],
        }

    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> "ReplayRecording":
        return cls(
            frames=[FrameSnapshot.from_json(frame) for frame in payload.get("frames", [])],
            metadata=dict(payload.get("metadata", {})),
        )


class ReplayRecorder:
    """Capture frame-by-frame replay data."""

    def __init__(self, fps: float, metadata: dict[str, Any] | None = None) -> None:
        self.fps = fps
        self.frame_interval = 0.0 if fps <= 0.0 else 1.0 / fps
        self.metadata = metadata or {}
        self.frames: list[FrameSnapshot] = []
        self._last_capture_time: float | None = None

    def reset(self) -> None:
        self.frames.clear()
        self._last_capture_time = None

    def capture(self, bodies: Iterable[RigidBody], time: float, contact_count: int) -> None:
        if self.frame_interval <= 0.0:
            self.frames.append(FrameSnapshot.from_world(bodies, time, contact_count))
            return
        if self._last_capture_time is None or (time - self._last_capture_time) >= self.frame_interval:
            self.frames.append(FrameSnapshot.from_world(bodies, time, contact_count))
            self._last_capture_time = time

    def build_recording(self) -> ReplayRecording:
        return ReplayRecording(frames=list(self.frames), metadata=dict(self.metadata))


def save_recording(recording: ReplayRecording, path: str | Path) -> Path:
    recording_path = Path(path)
    recording_path.write_text(json.dumps(recording.to_json(), indent=2, sort_keys=True))
    return recording_path


def load_recording(path: str | Path) -> ReplayRecording:
    payload = json.loads(Path(path).read_text())
    if not isinstance(payload, dict):
        raise ValueError("Replay JSON must be an object.")
    return ReplayRecording.from_json(payload)


def sample_contact_count(bodies: Iterable[RigidBody], ground_z: float, ground_body: RigidBody) -> int:
    contacts = detect_contacts(list(bodies), ground_z, ground_body)
    return len(contacts)
