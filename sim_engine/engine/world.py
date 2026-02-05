from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Callable, List

from .collision import detect_contacts
from .materials import Material
from .rigid_body import BodyType, RigidBody, Vec2
from .shapes import Box
from .solver import SequentialImpulseSolver


@dataclass
class World:
    fixed_dt: float = 1.0 / 60.0
    gravity: Vec2 = field(default_factory=lambda: Vec2(0.0, -9810.0))
    solver_iterations: int = 10
    ground_z: float = 0.0
    seed: int | None = None

    def __post_init__(self) -> None:
        self.bodies: List[RigidBody] = []
        self.accumulator = 0.0
        self.rng = random.Random(self.seed)
        self.ground_body = RigidBody(
            shape=Box(0.0, 0.0),
            body_type=BodyType.STATIC,
            position=Vec2(0.0, self.ground_z),
            material=Material(),
        )
        self.solver = SequentialImpulseSolver(self.solver_iterations)

    def add_body(self, body: RigidBody) -> None:
        self.bodies.append(body)

    def step(self, dt: float, on_step: Callable[[float], None] | None = None) -> None:
        self.accumulator += dt
        while self.accumulator >= self.fixed_dt:
            self._step_fixed(self.fixed_dt)
            if on_step is not None:
                on_step(self.fixed_dt)
            self.accumulator -= self.fixed_dt

    def _step_fixed(self, dt: float) -> None:
        for body in self.bodies:
            if body.body_type == BodyType.DYNAMIC:
                acceleration = self.gravity + (body.force * body.inv_mass)
                body.velocity = body.velocity + acceleration * dt
                body.angular_velocity += body.torque * body.inv_inertia * dt
            body.clear_forces()

        contacts = detect_contacts(self.bodies, self.ground_z, self.ground_body)
        self.solver.solve(contacts)

        for body in self.bodies:
            if body.body_type in {BodyType.DYNAMIC, BodyType.KINEMATIC}:
                body.position = body.position + body.velocity * dt
                body.angle += body.angular_velocity * dt
