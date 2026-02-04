"""Coordinate transforms for the hexapod simulation."""

from __future__ import annotations

import math

from . import config

EPS = 1e-9

HIP_POS_B = config.hip_pos_B
HIP_YAW0 = config.hip_yaw0


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def rot_z(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        (c, -s, 0.0),
        (s, c, 0.0),
        (0.0, 0.0, 1.0),
    )


def rot_y(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        (c, 0.0, s),
        (0.0, 1.0, 0.0),
        (-s, 0.0, c),
    )


def rot_x(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        (1.0, 0.0, 0.0),
        (0.0, c, -s),
        (0.0, s, c),
    )


def mat_mul(A, B):
    return tuple(
        tuple(
            A[r][0] * B[0][c] + A[r][1] * B[1][c] + A[r][2] * B[2][c]
            for c in range(3)
        )
        for r in range(3)
    )


def mat_vec(M, v):
    return (
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
    )


def mat_T(M):
    return (
        (M[0][0], M[1][0], M[2][0]),
        (M[0][1], M[1][1], M[2][1]),
        (M[0][2], M[1][2], M[2][2]),
    )


def R_body(roll=0.0, pitch=0.0, yaw=0.0):
    return mat_mul(rot_z(yaw), mat_mul(rot_y(pitch), rot_x(roll)))


def body_vec_to_hip(leg_idx, v_B):
    i = int(leg_idx)
    return mat_vec(rot_z(-HIP_YAW0[i]), v_B)


def hip_vec_to_body(leg_idx, v_H):
    i = int(leg_idx)
    return mat_vec(rot_z(HIP_YAW0[i]), v_H)


def body_point_to_hip(leg_idx, p_B):
    i = int(leg_idx)
    rel_B = v_sub(p_B, HIP_POS_B[i])
    return mat_vec(rot_z(-HIP_YAW0[i]), rel_B)


def hip_point_to_body(leg_idx, p_H):
    i = int(leg_idx)
    rel_B = mat_vec(rot_z(HIP_YAW0[i]), p_H)
    return v_add(HIP_POS_B[i], rel_B)


def world_point_to_body(p_W, body_pos_W=(0.0, 0.0, 0.0), body_rpy=(0.0, 0.0, 0.0)):
    roll, pitch, yaw = body_rpy
    R = R_body(roll, pitch, yaw)
    rel = v_sub(p_W, body_pos_W)
    return mat_vec(mat_T(R), rel)


def body_point_to_world(p_B, body_pos_W=(0.0, 0.0, 0.0), body_rpy=(0.0, 0.0, 0.0)):
    roll, pitch, yaw = body_rpy
    R = R_body(roll, pitch, yaw)
    return v_add(body_pos_W, mat_vec(R, p_B))


def foot_target_body_to_hip(leg_idx, foot_B):
    return body_point_to_hip(leg_idx, foot_B)
