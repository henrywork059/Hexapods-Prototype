"""Input state for the desktop viewer."""

from __future__ import annotations

from dataclasses import dataclass

from .camera import Camera2D


@dataclass
class InputState:
    camera: Camera2D
    is_panning: bool = False
    last_mouse: tuple[float, float] | None = None

    def start_pan(self, x: float, y: float) -> None:
        self.is_panning = True
        self.last_mouse = (x, y)

    def end_pan(self) -> None:
        self.is_panning = False
        self.last_mouse = None

    def pan_to(self, x: float, y: float) -> None:
        if not self.is_panning or self.last_mouse is None:
            return
        last_x, last_y = self.last_mouse
        self.camera.pan(x - last_x, y - last_y)
        self.last_mouse = (x, y)

    def zoom(self, factor: float, x: float, y: float) -> None:
        self.camera.zoom_at(factor, (x, y))
