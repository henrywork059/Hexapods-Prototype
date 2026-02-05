from __future__ import annotations

from typing import Iterable

from .contacts import Contact
from .materials import combine_friction, combine_restitution
from .rigid_body import Vec2, cross_scalar_vec, cross_vec_vec


class SequentialImpulseSolver:
    def __init__(self, iterations: int = 10) -> None:
        self.iterations = iterations

    def solve(self, contacts: Iterable[Contact]) -> None:
        contacts_list = list(contacts)
        for _ in range(self.iterations):
            for contact in contacts_list:
                self._solve_contact(contact)
        self._positional_correction(contacts_list)

    def _solve_contact(self, contact: Contact) -> None:
        body_a = contact.body_a
        body_b = contact.body_b
        normal = contact.normal
        ra = contact.point - body_a.position
        rb = contact.point - body_b.position

        vel_a = body_a.velocity + cross_scalar_vec(body_a.angular_velocity, ra)
        vel_b = body_b.velocity + cross_scalar_vec(body_b.angular_velocity, rb)
        relative_vel = vel_b - vel_a

        vel_along_normal = relative_vel.dot(normal)
        if vel_along_normal > 0.0:
            return

        restitution = combine_restitution(body_a.material, body_b.material)
        inv_mass_sum = body_a.inv_mass + body_b.inv_mass
        inv_mass_sum += (cross_vec_vec(ra, normal) ** 2) * body_a.inv_inertia
        inv_mass_sum += (cross_vec_vec(rb, normal) ** 2) * body_b.inv_inertia
        if inv_mass_sum == 0.0:
            return

        j = -(1.0 + restitution) * vel_along_normal
        j /= inv_mass_sum
        impulse = normal * j
        body_a.apply_impulse(impulse * -1.0, ra)
        body_b.apply_impulse(impulse, rb)

        tangent = (relative_vel - normal * vel_along_normal).normalized()
        vt = relative_vel.dot(tangent)
        if tangent.length() == 0.0:
            return
        jt = -vt
        jt /= inv_mass_sum
        friction = combine_friction(body_a.material, body_b.material)
        jt = max(-j * friction, min(jt, j * friction))
        friction_impulse = tangent * jt
        body_a.apply_impulse(friction_impulse * -1.0, ra)
        body_b.apply_impulse(friction_impulse, rb)

    def _positional_correction(self, contacts: Iterable[Contact]) -> None:
        percent = 0.2
        slop = 0.5
        for contact in contacts:
            body_a = contact.body_a
            body_b = contact.body_b
            inv_mass_sum = body_a.inv_mass + body_b.inv_mass
            if inv_mass_sum == 0.0:
                continue
            correction_mag = max(contact.penetration - slop, 0.0) / inv_mass_sum * percent
            correction = contact.normal * correction_mag
            if body_a.inv_mass > 0.0:
                body_a.position = body_a.position - correction * body_a.inv_mass
            if body_b.inv_mass > 0.0:
                body_b.position = body_b.position + correction * body_b.inv_mass
