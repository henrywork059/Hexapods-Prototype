"""Schema and helpers for GUI-configurable simulation settings."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from . import config


@dataclass
class SimSettings:
    scene: str = config.DEFAULT_SCENE
    fixed_dt: float = config.DEFAULT_FIXED_DT
    fps: float = config.DEFAULT_FPS
    gravity_x: float = config.DEFAULT_GRAVITY_X
    gravity_z: float = config.DEFAULT_GRAVITY_Z
    solver_iterations: int = config.DEFAULT_SOLVER_ITERATIONS
    mu_static: float = config.DEFAULT_MU_STATIC
    mu_dynamic: float = config.DEFAULT_MU_DYNAMIC
    restitution: float = config.DEFAULT_RESTITUTION
    ground_z: float = config.DEFAULT_GROUND_Z
    seed: int | None = config.DEFAULT_SEED
    window_width: int = config.DEFAULT_WINDOW_WIDTH
    window_height: int = config.DEFAULT_WINDOW_HEIGHT
    zoom: float = config.DEFAULT_ZOOM
    show_axes: bool = config.DEFAULT_SHOW_AXES
    show_contacts: bool = config.DEFAULT_SHOW_CONTACTS
    show_stats: bool = config.DEFAULT_SHOW_STATS
    recording_enabled: bool = config.DEFAULT_RECORDING_ENABLED
    recording_path: str = config.DEFAULT_RECORDING_PATH
    recording_fps: float = config.DEFAULT_RECORDING_FPS
    playback_enabled: bool = config.DEFAULT_PLAYBACK_ENABLED
    playback_path: str = config.DEFAULT_PLAYBACK_PATH

    def to_json(self) -> dict[str, Any]:
        return {
            "scene": self.scene,
            "fixed_dt": self.fixed_dt,
            "fps": self.fps,
            "gravity_x": self.gravity_x,
            "gravity_z": self.gravity_z,
            "solver_iterations": self.solver_iterations,
            "mu_static": self.mu_static,
            "mu_dynamic": self.mu_dynamic,
            "restitution": self.restitution,
            "ground_z": self.ground_z,
            "seed": self.seed,
            "window_width": self.window_width,
            "window_height": self.window_height,
            "zoom": self.zoom,
            "show_axes": self.show_axes,
            "show_contacts": self.show_contacts,
            "show_stats": self.show_stats,
            "recording_enabled": self.recording_enabled,
            "recording_path": self.recording_path,
            "recording_fps": self.recording_fps,
            "playback_enabled": self.playback_enabled,
            "playback_path": self.playback_path,
        }

    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> "SimSettings":
        return cls(
            scene=str(payload.get("scene", config.DEFAULT_SCENE)),
            fixed_dt=float(payload.get("fixed_dt", config.DEFAULT_FIXED_DT)),
            fps=float(payload.get("fps", config.DEFAULT_FPS)),
            gravity_x=float(payload.get("gravity_x", config.DEFAULT_GRAVITY_X)),
            gravity_z=float(payload.get("gravity_z", payload.get("gravity_y", config.DEFAULT_GRAVITY_Z))),
            solver_iterations=int(payload.get("solver_iterations", config.DEFAULT_SOLVER_ITERATIONS)),
            mu_static=float(payload.get("mu_static", config.DEFAULT_MU_STATIC)),
            mu_dynamic=float(payload.get("mu_dynamic", config.DEFAULT_MU_DYNAMIC)),
            restitution=float(payload.get("restitution", config.DEFAULT_RESTITUTION)),
            ground_z=float(payload.get("ground_z", config.DEFAULT_GROUND_Z)),
            seed=payload.get("seed", config.DEFAULT_SEED),
            window_width=int(payload.get("window_width", config.DEFAULT_WINDOW_WIDTH)),
            window_height=int(payload.get("window_height", config.DEFAULT_WINDOW_HEIGHT)),
            zoom=float(payload.get("zoom", config.DEFAULT_ZOOM)),
            show_axes=bool(payload.get("show_axes", config.DEFAULT_SHOW_AXES)),
            show_contacts=bool(payload.get("show_contacts", config.DEFAULT_SHOW_CONTACTS)),
            show_stats=bool(payload.get("show_stats", config.DEFAULT_SHOW_STATS)),
            recording_enabled=bool(payload.get("recording_enabled", config.DEFAULT_RECORDING_ENABLED)),
            recording_path=str(payload.get("recording_path", config.DEFAULT_RECORDING_PATH)),
            recording_fps=float(payload.get("recording_fps", config.DEFAULT_RECORDING_FPS)),
            playback_enabled=bool(payload.get("playback_enabled", config.DEFAULT_PLAYBACK_ENABLED)),
            playback_path=str(payload.get("playback_path", config.DEFAULT_PLAYBACK_PATH)),
        )


def load_last_used(path: Path | None = None) -> SimSettings:
    settings_path = path or config.DEFAULT_SETTINGS_PATH
    try:
        data = json.loads(settings_path.read_text())
    except FileNotFoundError:
        return SimSettings()
    except json.JSONDecodeError:
        return SimSettings()
    return SimSettings.from_json(data if isinstance(data, dict) else {})


def save_last_used(settings: SimSettings, path: Path | None = None) -> Path:
    settings_path = path or config.DEFAULT_SETTINGS_PATH
    settings_path.write_text(json.dumps(settings.to_json(), indent=2, sort_keys=True))
    return settings_path
