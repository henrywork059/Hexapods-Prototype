import unittest

from sim_engine.engine.rigid_body import RigidBody, Vec2
from sim_engine.engine.shapes import Circle
from sim_engine.engine.world import World


class StabilityTests(unittest.TestCase):
    def test_fixed_dt_prevents_tunneling(self) -> None:
        radius = 10.0
        world = World(fixed_dt=1.0 / 240.0, solver_iterations=20, gravity=Vec2(0.0, 0.0))
        body = RigidBody(shape=Circle(radius), position=Vec2(0.0, 100.0), velocity=Vec2(0.0, -2000.0))
        world.add_body(body)
        for _ in range(240):
            world.step(1.0 / 240.0)
        self.assertGreaterEqual(body.position.z, radius - 0.6)


if __name__ == "__main__":
    unittest.main()
