"""Default configuration values for sim_engine GUI settings."""

from __future__ import annotations

from pathlib import Path

DEFAULT_FIXED_DT = 1.0 / 60.0
DEFAULT_GRAVITY_X = 0.0
DEFAULT_GRAVITY_Y = -9810.0
DEFAULT_SOLVER_ITERATIONS = 10
DEFAULT_GROUND_Z = 0.0
DEFAULT_SEED = None

DEFAULT_SETTINGS_PATH = Path.home() / ".sim_engine_settings.json"
