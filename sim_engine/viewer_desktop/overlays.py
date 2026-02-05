"""Rendering overlays for the desktop viewer."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import tkinter as tk

from sim_engine.engine.collision import box_vertices
from sim_engine.engine.contacts import Contact
from sim_engine.engine.rigid_body import BodyType, RigidBody, Vec2
from sim_engine.engine.shapes import Box, Circle

from .camera import Camera2D


@dataclass
class OverlayColors:
    ground: str = "#999999"
    static_body: str = "#4C7AEF"
    dynamic_body: str = "#56C271"
    outline: str = "#1E1E1E"
    contact_point: str = "#FF5533"
    contact_normal: str = "#FFAA33"
    velocity: str = "#33A1FF"


def draw_ground(canvas: tk.Canvas, camera: Camera2D, ground_z: float, width: int) -> None:
    left = camera.screen_to_world(0.0, camera.height * 0.5)
    right = camera.screen_to_world(float(width), camera.height * 0.5)
    start = camera.world_to_screen(Vec2(left.x, ground_z))
    end = camera.world_to_screen(Vec2(right.x, ground_z))
    canvas.create_line(*start, *end, fill=OverlayColors().ground, width=2)


def draw_body(canvas: tk.Canvas, camera: Camera2D, body: RigidBody, colors: OverlayColors) -> None:
    if isinstance(body.shape, Circle):
        draw_circle(canvas, camera, body, colors)
    elif isinstance(body.shape, Box):
        draw_box(canvas, camera, body, colors)


def draw_circle(canvas: tk.Canvas, camera: Camera2D, body: RigidBody, colors: OverlayColors) -> None:
    radius = body.shape.radius
    center = camera.world_to_screen(body.position)
    scale = camera.pixels_per_mm * camera.zoom
    r_px = radius * scale
    fill = colors.static_body if body.body_type == BodyType.STATIC else colors.dynamic_body
    canvas.create_oval(
        center[0] - r_px,
        center[1] - r_px,
        center[0] + r_px,
        center[1] + r_px,
        fill=fill,
        outline="",
    )
    canvas.create_oval(
        center[0] - r_px,
        center[1] - r_px,
        center[0] + r_px,
        center[1] + r_px,
        outline=colors.outline,
        width=2,
    )
    marker = camera.world_to_screen(
        Vec2(
            body.position.x + math.cos(body.angle) * radius,
            body.position.z + math.sin(body.angle) * radius,
        )
    )
    canvas.create_line(*center, *marker, fill=colors.outline, width=2)


def draw_box(canvas: tk.Canvas, camera: Camera2D, body: RigidBody, colors: OverlayColors) -> None:
    vertices = box_vertices(body, body.shape)
    points = []
    for vertex in vertices:
        points.extend(camera.world_to_screen(vertex))
    fill = colors.static_body if body.body_type == BodyType.STATIC else colors.dynamic_body
    canvas.create_polygon(*points, fill=fill, outline="")
    canvas.create_polygon(*points, outline=colors.outline, fill="", width=2)


def draw_contacts(
    canvas: tk.Canvas,
    camera: Camera2D,
    contacts: Iterable[Contact],
    colors: OverlayColors,
    normal_scale: float = 15.0,
) -> None:
    for contact in contacts:
        point = camera.world_to_screen(contact.point)
        canvas.create_oval(
            point[0] - 3,
            point[1] - 3,
            point[0] + 3,
            point[1] + 3,
            fill=colors.contact_point,
            outline="",
        )
        normal_end = camera.world_to_screen(
            Vec2(
                contact.point.x + contact.normal.x * normal_scale,
                contact.point.z + contact.normal.z * normal_scale,
            )
        )
        canvas.create_line(*point, *normal_end, fill=colors.contact_normal, width=2)


def draw_velocity_vectors(
    canvas: tk.Canvas,
    camera: Camera2D,
    bodies: Iterable[RigidBody],
    colors: OverlayColors,
    velocity_scale: float = 0.02,
) -> None:
    for body in bodies:
        if body.body_type == BodyType.STATIC:
            continue
        origin = camera.world_to_screen(body.position)
        tip = camera.world_to_screen(
            Vec2(
                body.position.x + body.velocity.x * velocity_scale,
                body.position.z + body.velocity.z * velocity_scale,
            )
        )
        canvas.create_line(*origin, *tip, fill=colors.velocity, width=2, arrow=tk.LAST)
