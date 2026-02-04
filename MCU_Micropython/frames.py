\
# frames.py
# Coordinate transforms for the hexapod.
#
# Conventions (matches your project notes):
# - Body frame B: origin at body center
#     +x_B forward
#     +y_B left
#     +z_B up
#
# - Each leg has its own hip-local frame H_i:
#     origin at the hip joint (hip is 0,0,0)
#     +x_H radial outward from body center through that hip
#     +y_H left (leg-local)
#     +z_H up  (so foot under hip has negative z)
#
# hip_pos_B[i]   : hip position of leg i in body frame (mm)
# hip_yaw0[i]    : rotation (rad) from body frame to hip-local frame about +z
#                  (typically atan2(y, x) of hip_pos_B)
#
# This module does NOT require world coordinates. It mainly supports:
#   body-frame <-> hip-frame conversions.

import math

# --- config import (supports either config.py or config_pca.py) ---
try:
    import config
except ImportError:
    import config_pca as config  # fallback if you didn't rename config_pca.py


EPS = 1e-9


def _get_hip_pos_B():
    hip_pos = getattr(config, "hip_pos_B", None)
    if hip_pos is None:
        raise RuntimeError("config.hip_pos_B is missing. Define hip_pos_B in config.")
    if len(hip_pos) != 6:
        raise RuntimeError("config.hip_pos_B must have 6 entries (one per leg).")
    return hip_pos


def _get_hip_yaw0():
    yaw0 = getattr(config, "hip_yaw0", None)
    if yaw0 is not None and len(yaw0) == 6:
        return yaw0

    # Derive if not provided
    hip_pos = _get_hip_pos_B()
    return [math.atan2(y, x) for (x, y, _z) in hip_pos]


HIP_POS_B = _get_hip_pos_B()
HIP_YAW0 = _get_hip_yaw0()


# -----------------------------
# Small linear algebra helpers
# -----------------------------
def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def rot_z(theta):
    """Rotation matrix Rz(theta) as 3 tuples (row-major)."""
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        (c, -s, 0.0),
        (s,  c, 0.0),
        (0.0, 0.0, 1.0),
    )


def rot_y(theta):
    """Rotation matrix Ry(theta) as 3 tuples (row-major)."""
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        ( c, 0.0, s),
        (0.0, 1.0, 0.0),
        (-s, 0.0, c),
    )


def rot_x(theta):
    """Rotation matrix Rx(theta) as 3 tuples (row-major)."""
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        (1.0, 0.0, 0.0),
        (0.0,  c, -s),
        (0.0,  s,  c),
    )


def mat_mul(A, B):
    """3x3 * 3x3."""
    return tuple(
        tuple(
            A[r][0] * B[0][c] + A[r][1] * B[1][c] + A[r][2] * B[2][c]
            for c in range(3)
        )
        for r in range(3)
    )


def mat_vec(M, v):
    """3x3 * vec3."""
    return (
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
    )


def mat_T(M):
    """Transpose of a 3x3."""
    return (
        (M[0][0], M[1][0], M[2][0]),
        (M[0][1], M[1][1], M[2][1]),
        (M[0][2], M[1][2], M[2][2]),
    )


# -----------------------------
# Body orientation (optional)
# -----------------------------
def R_body(roll=0.0, pitch=0.0, yaw=0.0):
    """Return rotation matrix for body orientation using ZYX convention.

    R = Rz(yaw) * Ry(pitch) * Rx(roll)

    Useful later if you add a world frame. For purely body<->hip transforms,
    you can ignore this.
    """
    return mat_mul(rot_z(yaw), mat_mul(rot_y(pitch), rot_x(roll)))


# -----------------------------
# Core hip/body transforms
# -----------------------------
def body_vec_to_hip(leg_idx, v_B):
    """Rotate a vector from body frame into hip-local frame (no translation)."""
    i = int(leg_idx)
    return mat_vec(rot_z(-HIP_YAW0[i]), v_B)


def hip_vec_to_body(leg_idx, v_H):
    """Rotate a vector from hip-local frame into body frame (no translation)."""
    i = int(leg_idx)
    return mat_vec(rot_z(HIP_YAW0[i]), v_H)


def body_point_to_hip(leg_idx, p_B):
    """Convert a point in body frame to hip-local coordinates.

    p_H = Rz(-hip_yaw0) * (p_B - hip_pos_B)
    """
    i = int(leg_idx)
    rel_B = v_sub(p_B, HIP_POS_B[i])
    return mat_vec(rot_z(-HIP_YAW0[i]), rel_B)


def hip_point_to_body(leg_idx, p_H):
    """Convert a point in hip-local frame to body coordinates.

    p_B = hip_pos_B + Rz(+hip_yaw0) * p_H
    """
    i = int(leg_idx)
    rel_B = mat_vec(rot_z(HIP_YAW0[i]), p_H)
    return v_add(HIP_POS_B[i], rel_B)


# -----------------------------
# Optional: world transforms (small, for later)
# -----------------------------
def world_point_to_body(p_W, body_pos_W=(0.0, 0.0, 0.0), body_rpy=(0.0, 0.0, 0.0)):
    """Convert a world point into body frame.

    p_B = R^T * (p_W - body_pos_W)
    where R = R_body(roll,pitch,yaw)
    """
    roll, pitch, yaw = body_rpy
    R = R_body(roll, pitch, yaw)
    rel = v_sub(p_W, body_pos_W)
    return mat_vec(mat_T(R), rel)


def body_point_to_world(p_B, body_pos_W=(0.0, 0.0, 0.0), body_rpy=(0.0, 0.0, 0.0)):
    """Convert a body-frame point into world frame.

    p_W = body_pos_W + R * p_B
    """
    roll, pitch, yaw = body_rpy
    R = R_body(roll, pitch, yaw)
    return v_add(body_pos_W, mat_vec(R, p_B))


# -----------------------------
# Convenience: body target -> hip target for IK
# -----------------------------
def foot_target_body_to_hip(leg_idx, foot_B):
    """Given a desired foot position expressed in BODY frame,
    return the hip-local (x,y,z) for that leg to pass into IK.
    """
    return body_point_to_hip(leg_idx, foot_B)
