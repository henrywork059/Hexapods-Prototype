from __future__ import annotations

import math
from typing import List

from .contacts import Contact
from .rigid_body import BodyType, RigidBody, Vec2
from .shapes import Box, Circle


def rotate(v: Vec2, angle: float) -> Vec2:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return Vec2(v.x * cos_a - v.z * sin_a, v.x * sin_a + v.z * cos_a)


def world_to_local(body: RigidBody, point: Vec2) -> Vec2:
    return rotate(point - body.position, -body.angle)


def local_to_world(body: RigidBody, point: Vec2) -> Vec2:
    return rotate(point, body.angle) + body.position


def box_vertices(body: RigidBody, shape: Box) -> List[Vec2]:
    hx, hz = shape.half_width, shape.half_height
    local = [Vec2(-hx, -hz), Vec2(hx, -hz), Vec2(hx, hz), Vec2(-hx, hz)]
    return [local_to_world(body, v) for v in local]


def point_in_box(body: RigidBody, shape: Box, point: Vec2) -> bool:
    local = world_to_local(body, point)
    return abs(local.x) <= shape.half_width and abs(local.z) <= shape.half_height


def circle_ground(circle_body: RigidBody, shape: Circle, ground_z: float, ground_body: RigidBody) -> List[Contact]:
    depth = ground_z - (circle_body.position.z - shape.radius)
    if depth <= 0.0:
        return []
    point = Vec2(circle_body.position.x, ground_z)
    return [Contact(circle_body, ground_body, point, Vec2(0.0, 1.0), depth)]


def box_ground(box_body: RigidBody, shape: Box, ground_z: float, ground_body: RigidBody) -> List[Contact]:
    vertices = box_vertices(box_body, shape)
    min_vertex = min(vertices, key=lambda v: v.z)
    depth = ground_z - min_vertex.z
    if depth <= 0.0:
        return []
    point = Vec2(min_vertex.x, ground_z)
    return [Contact(box_body, ground_body, point, Vec2(0.0, 1.0), depth)]


def circle_circle(body_a: RigidBody, shape_a: Circle, body_b: RigidBody, shape_b: Circle) -> List[Contact]:
    delta = body_b.position - body_a.position
    dist = delta.length()
    radius_sum = shape_a.radius + shape_b.radius
    if dist >= radius_sum or dist == 0.0:
        if dist == 0.0 and radius_sum > 0.0:
            normal = Vec2(1.0, 0.0)
            point = body_a.position
            return [Contact(body_a, body_b, point, normal, radius_sum)]
        return []
    normal = delta / dist
    point = body_a.position + normal * shape_a.radius
    return [Contact(body_a, body_b, point, normal, radius_sum - dist)]


def circle_box(circle_body: RigidBody, circle: Circle, box_body: RigidBody, box: Box) -> List[Contact]:
    local_center = world_to_local(box_body, circle_body.position)
    clamped = Vec2(
        max(-box.half_width, min(local_center.x, box.half_width)),
        max(-box.half_height, min(local_center.z, box.half_height)),
    )
    closest_world = local_to_world(box_body, clamped)
    delta = circle_body.position - closest_world
    dist = delta.length()
    if dist > circle.radius:
        return []
    if dist == 0.0:
        dx = box.half_width - abs(local_center.x)
        dz = box.half_height - abs(local_center.z)
        if dx < dz:
            normal_local = Vec2(1.0 if local_center.x > 0 else -1.0, 0.0)
        else:
            normal_local = Vec2(0.0, 1.0 if local_center.z > 0 else -1.0)
        normal = rotate(normal_local, box_body.angle)
        point = circle_body.position - normal * circle.radius
        return [Contact(circle_body, box_body, point, normal, circle.radius)]
    normal = delta / dist
    point = closest_world
    return [Contact(circle_body, box_body, point, normal, circle.radius - dist)]


def project_vertices(vertices: List[Vec2], axis: Vec2) -> tuple[float, float]:
    dots = [v.dot(axis) for v in vertices]
    return min(dots), max(dots)


def box_box(body_a: RigidBody, shape_a: Box, body_b: RigidBody, shape_b: Box) -> List[Contact]:
    vertices_a = box_vertices(body_a, shape_a)
    vertices_b = box_vertices(body_b, shape_b)
    axes = [
        rotate(Vec2(1.0, 0.0), body_a.angle),
        rotate(Vec2(0.0, 1.0), body_a.angle),
        rotate(Vec2(1.0, 0.0), body_b.angle),
        rotate(Vec2(0.0, 1.0), body_b.angle),
    ]
    min_overlap = float("inf")
    collision_axis = None
    for axis in axes:
        axis = axis.normalized()
        min_a, max_a = project_vertices(vertices_a, axis)
        min_b, max_b = project_vertices(vertices_b, axis)
        overlap = min(max_a, max_b) - max(min_a, min_b)
        if overlap <= 0.0:
            return []
        if overlap < min_overlap:
            min_overlap = overlap
            collision_axis = axis
    if collision_axis is None:
        return []
    center_delta = body_b.position - body_a.position
    if center_delta.dot(collision_axis) < 0.0:
        collision_axis = collision_axis * -1.0
    contact_points: List[Vec2] = []
    for vertex in vertices_a:
        if point_in_box(body_b, shape_b, vertex):
            contact_points.append(vertex)
    for vertex in vertices_b:
        if point_in_box(body_a, shape_a, vertex):
            contact_points.append(vertex)
    if contact_points:
        avg_x = sum(p.x for p in contact_points) / len(contact_points)
        avg_z = sum(p.z for p in contact_points) / len(contact_points)
        point = Vec2(avg_x, avg_z)
    else:
        point = body_a.position + collision_axis * (min_overlap * 0.5)
    return [Contact(body_a, body_b, point, collision_axis, min_overlap)]


def detect_contacts(bodies: List[RigidBody], ground_z: float, ground_body: RigidBody) -> List[Contact]:
    contacts: List[Contact] = []
    for body in bodies:
        if body.body_type == BodyType.STATIC:
            continue
        if isinstance(body.shape, Circle):
            contacts.extend(circle_ground(body, body.shape, ground_z, ground_body))
        elif isinstance(body.shape, Box):
            contacts.extend(box_ground(body, body.shape, ground_z, ground_body))
    for i, body_a in enumerate(bodies):
        for body_b in bodies[i + 1 :]:
            if body_a.body_type == BodyType.STATIC and body_b.body_type == BodyType.STATIC:
                continue
            shape_a = body_a.shape
            shape_b = body_b.shape
            if isinstance(shape_a, Circle) and isinstance(shape_b, Circle):
                contacts.extend(circle_circle(body_a, shape_a, body_b, shape_b))
            elif isinstance(shape_a, Circle) and isinstance(shape_b, Box):
                contacts.extend(circle_box(body_a, shape_a, body_b, shape_b))
            elif isinstance(shape_a, Box) and isinstance(shape_b, Circle):
                contact_list = circle_box(body_b, shape_b, body_a, shape_a)
                for contact in contact_list:
                    contacts.append(Contact(contact.body_b, contact.body_a, contact.point, contact.normal * -1.0, contact.penetration))
            elif isinstance(shape_a, Box) and isinstance(shape_b, Box):
                contacts.extend(box_box(body_a, shape_a, body_b, shape_b))
    return contacts
