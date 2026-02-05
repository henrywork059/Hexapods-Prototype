import unittest

from sim_engine.engine.collision import (
    box_box,
    box_ground,
    circle_box,
    circle_circle,
    circle_ground,
    detect_contacts,
)
from sim_engine.engine.rigid_body import BodyType, RigidBody, Vec2
from sim_engine.engine.shapes import Box, Circle


class CollisionDetectionTests(unittest.TestCase):
    def test_circle_ground_contact(self) -> None:
        circle_body = RigidBody(shape=Circle(10.0), position=Vec2(0.0, 5.0))
        ground_body = RigidBody(shape=Box(0.0, 0.0), body_type=BodyType.STATIC)
        contacts = circle_ground(circle_body, circle_body.shape, 0.0, ground_body)
        self.assertEqual(len(contacts), 1)
        contact = contacts[0]
        self.assertAlmostEqual(contact.penetration, 5.0)
        self.assertEqual(contact.point, Vec2(0.0, 0.0))
        self.assertEqual(contact.normal, Vec2(0.0, -1.0))

    def test_box_ground_contact(self) -> None:
        box_body = RigidBody(shape=Box(4.0, 4.0), position=Vec2(0.0, 2.0))
        ground_body = RigidBody(shape=Box(0.0, 0.0), body_type=BodyType.STATIC)
        contacts = box_ground(box_body, box_body.shape, 0.0, ground_body)
        self.assertEqual(len(contacts), 1)
        self.assertAlmostEqual(contacts[0].penetration, 2.0)

    def test_circle_circle_contact(self) -> None:
        body_a = RigidBody(shape=Circle(5.0), position=Vec2(0.0, 0.0))
        body_b = RigidBody(shape=Circle(5.0), position=Vec2(8.0, 0.0))
        contacts = circle_circle(body_a, body_a.shape, body_b, body_b.shape)
        self.assertEqual(len(contacts), 1)
        contact = contacts[0]
        self.assertAlmostEqual(contact.penetration, 2.0)
        self.assertAlmostEqual(contact.normal.x, 1.0)
        self.assertAlmostEqual(contact.normal.z, 0.0)

    def test_circle_box_contact(self) -> None:
        circle_body = RigidBody(shape=Circle(3.0), position=Vec2(7.0, 0.0))
        box_body = RigidBody(shape=Box(5.0, 5.0), position=Vec2(0.0, 0.0))
        contacts = circle_box(circle_body, circle_body.shape, box_body, box_body.shape)
        self.assertEqual(len(contacts), 1)
        contact = contacts[0]
        self.assertGreater(contact.penetration, 0.0)
        self.assertGreater(contact.normal.x, 0.5)

    def test_box_box_contact(self) -> None:
        body_a = RigidBody(shape=Box(3.0, 3.0), position=Vec2(0.0, 0.0))
        body_b = RigidBody(shape=Box(3.0, 3.0), position=Vec2(4.0, 0.0))
        contacts = box_box(body_a, body_a.shape, body_b, body_b.shape)
        self.assertEqual(len(contacts), 1)
        self.assertGreater(contacts[0].penetration, 0.0)

    def test_detect_contacts_box_circle_order(self) -> None:
        box_body = RigidBody(shape=Box(5.0, 5.0), position=Vec2(0.0, 0.0))
        circle_body = RigidBody(shape=Circle(3.0), position=Vec2(7.0, 0.0))
        contacts = detect_contacts([box_body, circle_body], -1000.0, RigidBody(shape=Box(0.0, 0.0), body_type=BodyType.STATIC))
        self.assertEqual(len(contacts), 1)
        self.assertGreater(contacts[0].penetration, 0.0)
        self.assertAlmostEqual(contacts[0].normal.length(), 1.0)


if __name__ == "__main__":
    unittest.main()
