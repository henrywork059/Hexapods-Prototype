"""Scene factories for sim_engine demos."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from sim_engine.engine.materials import Material
from sim_engine.engine.rigid_body import BodyType, RigidBody, Vec2
from sim_engine.engine.shapes import Box, Circle
from sim_engine.engine.world import World
from sim_engine.settings_schema import SimSettings

SceneUpdater = Callable[[World, SimSettings, float, float], None]


@dataclass(frozen=True)
class SceneSpec:
    name: str
    builder: Callable[[World, SimSettings], SceneUpdater | None]


def _material_from_settings(settings: SimSettings) -> Material:
    return Material(friction=settings.mu_dynamic, restitution=settings.restitution)


def _spawn_box(
    world: World,
    settings: SimSettings,
    position: Vec2,
    size: tuple[float, float],
    angle: float = 0.0,
    velocity: Vec2 | None = None,
    body_type: BodyType = BodyType.DYNAMIC,
) -> RigidBody:
    body = RigidBody(
        shape=Box(size[0] * 0.5, size[1] * 0.5),
        body_type=body_type,
        position=position,
        angle=angle,
        material=_material_from_settings(settings),
    )
    if velocity is not None:
        body.velocity = velocity
    world.add_body(body)
    return body


def _spawn_circle(
    world: World,
    settings: SimSettings,
    position: Vec2,
    radius: float,
    velocity: Vec2 | None = None,
    body_type: BodyType = BodyType.DYNAMIC,
) -> RigidBody:
    body = RigidBody(
        shape=Circle(radius),
        body_type=body_type,
        position=position,
        material=_material_from_settings(settings),
    )
    if velocity is not None:
        body.velocity = velocity
    world.add_body(body)
    return body


def build_falling_settle_scene(world: World, settings: SimSettings) -> SceneUpdater | None:
    """Drop mixed bodies so they fall and settle."""
    world.ground_body.material = _material_from_settings(settings)
    for i in range(6):
        x = world.rng.uniform(-160.0, 160.0)
        z = 120.0 + i * 50.0
        if i % 2 == 0:
            _spawn_box(
                world,
                settings,
                Vec2(x, z),
                (world.rng.uniform(40.0, 70.0), world.rng.uniform(20.0, 40.0)),
                angle=world.rng.uniform(-0.3, 0.3),
            )
        else:
            _spawn_circle(world, settings, Vec2(x, z), world.rng.uniform(16.0, 26.0))
    return None


def build_friction_slide_scene(world: World, settings: SimSettings) -> SceneUpdater | None:
    """Send a box sliding; mu settings should affect the stopping distance."""
    world.ground_body.material = _material_from_settings(settings)
    _spawn_box(
        world,
        settings,
        Vec2(-220.0, 20.0),
        (80.0, 30.0),
        velocity=Vec2(520.0, 0.0),
    )
    _spawn_circle(
        world,
        settings,
        Vec2(-120.0, 18.0),
        14.0,
        velocity=Vec2(450.0, 0.0),
    )
    return None


def build_stack_collision_scene(world: World, settings: SimSettings) -> SceneUpdater | None:
    """Stacked blocks with a fast mover to knock them over."""
    world.ground_body.material = _material_from_settings(settings)
    for i in range(5):
        _spawn_box(world, settings, Vec2(140.0, 20.0 + i * 35.0), (60.0, 20.0))
    _spawn_circle(
        world,
        settings,
        Vec2(-200.0, 25.0),
        22.0,
        velocity=Vec2(780.0, 0.0),
    )
    return None


def build_hexapod_feet_scene(world: World, settings: SimSettings) -> SceneUpdater:
    """Placeholder hexapod feet: kinematic circles with scripted motion."""
    world.ground_body.material = _material_from_settings(settings)
    base_positions = [
        Vec2(-150.0, 18.0),
        Vec2(-90.0, 18.0),
        Vec2(-30.0, 18.0),
        Vec2(30.0, 18.0),
        Vec2(90.0, 18.0),
        Vec2(150.0, 18.0),
    ]
    phases = [0.0, math.pi, 0.0, math.pi, 0.0, math.pi]
    feet = [
        _spawn_circle(world, settings, position, 12.0, body_type=BodyType.KINEMATIC)
        for position in base_positions
    ]

    def _update(_world: World, _settings: SimSettings, time_s: float, dt: float) -> None:
        if dt <= 0.0:
            return
        stride = 45.0
        lift = 18.0
        omega = 2.0 * math.pi * 0.6
        for foot, base, phase in zip(feet, base_positions, phases):
            cycle = omega * time_s + phase
            swing = math.sin(cycle)
            lift_phase = max(0.0, math.sin(cycle))
            target = Vec2(base.x + stride * swing, base.z + lift * lift_phase)
            foot.velocity = (target - foot.position) / dt

    return _update


SCENES = [
    SceneSpec("falling_settle", build_falling_settle_scene),
    SceneSpec("friction_slide", build_friction_slide_scene),
    SceneSpec("stack_collision", build_stack_collision_scene),
    SceneSpec("hexapod_feet", build_hexapod_feet_scene),
]
