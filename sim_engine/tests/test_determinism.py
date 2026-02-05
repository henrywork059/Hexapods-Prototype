import unittest

from sim_engine.engine.rigid_body import RigidBody, Vec2
from sim_engine.engine.shapes import Box, Circle
from sim_engine.engine.world import World


def build_world(seed: int) -> World:
    world = World(seed=seed)
    for _ in range(3):
        if world.rng.random() > 0.5:
            shape = Circle(world.rng.uniform(5.0, 15.0))
        else:
            shape = Box(world.rng.uniform(3.0, 8.0), world.rng.uniform(3.0, 8.0))
        body = RigidBody(
            shape=shape,
            position=Vec2(world.rng.uniform(-50.0, 50.0), world.rng.uniform(20.0, 80.0)),
            velocity=Vec2(world.rng.uniform(-50.0, 50.0), world.rng.uniform(-50.0, 0.0)),
            angle=world.rng.uniform(-0.5, 0.5),
            angular_velocity=world.rng.uniform(-1.0, 1.0),
        )
        world.add_body(body)
    return world


class DeterminismTests(unittest.TestCase):
    def test_same_seed_same_state(self) -> None:
        world_a = build_world(123)
        world_b = build_world(123)
        for _ in range(120):
            world_a.step(1.0 / 60.0)
            world_b.step(1.0 / 60.0)
        for body_a, body_b in zip(world_a.bodies, world_b.bodies, strict=True):
            self.assertAlmostEqual(body_a.position.x, body_b.position.x)
            self.assertAlmostEqual(body_a.position.z, body_b.position.z)
            self.assertAlmostEqual(body_a.velocity.x, body_b.velocity.x)
            self.assertAlmostEqual(body_a.velocity.z, body_b.velocity.z)
            self.assertAlmostEqual(body_a.angle, body_b.angle)
            self.assertAlmostEqual(body_a.angular_velocity, body_b.angular_velocity)


if __name__ == "__main__":
    unittest.main()
