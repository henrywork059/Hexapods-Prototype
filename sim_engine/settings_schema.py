"""Schema and helpers for GUI-configurable simulation settings."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from . import config


@dataclass
class SimSettings:
    fixed_dt: float = config.DEFAULT_FIXED_DT
    gravity_x: float = config.DEFAULT_GRAVITY_X
    gravity_y: float = config.DEFAULT_GRAVITY_Y
    solver_iterations: int = config.DEFAULT_SOLVER_ITERATIONS
    ground_z: float = config.DEFAULT_GROUND_Z
    seed: int | None = config.DEFAULT_SEED

    def to_json(self) -> dict[str, Any]:
        return {
            "fixed_dt": self.fixed_dt,
            "gravity_x": self.gravity_x,
            "gravity_y": self.gravity_y,
            "solver_iterations": self.solver_iterations,
            "ground_z": self.ground_z,
            "seed": self.seed,
        }

    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> "SimSettings":
        return cls(
            fixed_dt=float(payload.get("fixed_dt", config.DEFAULT_FIXED_DT)),
            gravity_x=float(payload.get("gravity_x", config.DEFAULT_GRAVITY_X)),
            gravity_y=float(payload.get("gravity_y", config.DEFAULT_GRAVITY_Y)),
            solver_iterations=int(payload.get("solver_iterations", config.DEFAULT_SOLVER_ITERATIONS)),
            ground_z=float(payload.get("ground_z", config.DEFAULT_GROUND_Z)),
            seed=payload.get("seed", config.DEFAULT_SEED),
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
