import unittest

from sim_engine.math.vec2 import Vec2


class Vec2MathOpsTests(unittest.TestCase):
    def test_add_sub_mul_div(self) -> None:
        a = Vec2(3.0, -4.0)
        b = Vec2(1.5, 2.0)
        self.assertEqual(a + b, Vec2(4.5, -2.0))
        self.assertEqual(a - b, Vec2(1.5, -6.0))
        self.assertEqual(a * 2.0, Vec2(6.0, -8.0))
        self.assertEqual(2.0 * a, Vec2(6.0, -8.0))
        self.assertEqual(a / 2.0, Vec2(1.5, -2.0))

    def test_dot_cross_and_magnitude(self) -> None:
        a = Vec2(3.0, 4.0)
        b = Vec2(-2.0, 5.0)
        self.assertEqual(a.dot(b), 14.0)
        self.assertEqual(a.cross(b), 23.0)
        self.assertAlmostEqual(a.magnitude(), 5.0)

    def test_normalize(self) -> None:
        v = Vec2(3.0, 4.0)
        normalized = v.normalize()
        self.assertAlmostEqual(normalized.x, 0.6)
        self.assertAlmostEqual(normalized.y, 0.8)
        with self.assertRaises(ValueError):
            Vec2(0.0, 0.0).normalize()

    def test_divide_by_zero(self) -> None:
        with self.assertRaises(ValueError):
            Vec2(1.0, 2.0) / 0.0


if __name__ == "__main__":
    unittest.main()
