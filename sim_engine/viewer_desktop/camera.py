"""Camera utilities for the desktop viewer."""

from __future__ import annotations

from dataclasses import dataclass, field

from sim_engine.engine.rigid_body import Vec2


@dataclass
class Camera2D:
    """Simple 2D camera for world-to-screen transforms."""

    width: int
    height: int
    zoom: float = 1.0
    center: Vec2 = field(default_factory=lambda: Vec2(0.0, 0.0))
    pixels_per_mm: float = 2.0

    def world_to_screen(self, point: Vec2) -> tuple[float, float]:
        scale = self.pixels_per_mm * self.zoom
        x = (point.x - self.center.x) * scale + self.width * 0.5
        y = self.height * 0.5 - (point.z - self.center.z) * scale
        return x, y

    def screen_to_world(self, x: float, y: float) -> Vec2:
        scale = self.pixels_per_mm * self.zoom
        world_x = (x - self.width * 0.5) / scale + self.center.x
        world_z = (self.height * 0.5 - y) / scale + self.center.z
        return Vec2(world_x, world_z)

    def pan(self, dx_pixels: float, dy_pixels: float) -> None:
        scale = self.pixels_per_mm * self.zoom
        self.center = Vec2(
            self.center.x - dx_pixels / scale,
            self.center.z + dy_pixels / scale,
        )

    def zoom_at(self, factor: float, anchor: tuple[float, float]) -> None:
        if factor == 1.0:
            return
        anchor_world = self.screen_to_world(*anchor)
        self.zoom = max(0.05, min(self.zoom * factor, 50.0))
        updated_world = self.screen_to_world(*anchor)
        delta = Vec2(anchor_world.x - updated_world.x, anchor_world.z - updated_world.z)
        self.center = Vec2(self.center.x + delta.x, self.center.z + delta.z)

    def resize(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
