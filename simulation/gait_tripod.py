"""Tripod gait scheduler for simulation (mirrors real control flow)."""

from __future__ import annotations

import math

from . import config
from . import frames
from . import trajectory
from . import ik


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_norm2_xy(v):
    return math.hypot(v[0], v[1])


def rotz(theta, p):
    c = math.cos(theta)
    s = math.sin(theta)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1], p[2])


class TripodGait:
    """Tripod gait controller.

    Public API:
      - set_command(vx, vy, wz)   [mm/s, mm/s, rad/s] in BODY frame
      - update(dt=None)          returns list of 6 hip-local foot targets
      - set_enabled(True/False)
      - reset_pose(neutral=None)
    """

    def __init__(self):
        self.period = float(getattr(config, "GAIT_PERIOD", 0.8))
        self.phase_T = 0.5 * self.period

        self.swing_style = str(getattr(config, "SWING_STYLE", "sine"))
        self.swing_ease = str(getattr(config, "SWING_EASE", "quintic"))
        self.stance_ease = str(getattr(config, "STANCE_EASE", "linear"))

        self.clearance_base = float(getattr(config, "SWING_CLEARANCE", 25.0))
        self.clearance_gain = float(getattr(config, "SWING_CLEARANCE_GAIN", 0.15))
        self.clearance_max = float(getattr(config, "SWING_CLEARANCE_MAX", 50.0))

        self.cmd_deadband = float(getattr(config, "CMD_DEADBAND", 0.5))
        self.step_limit = float(getattr(config, "STEP_LIMIT_MM", 40.0))

        self.tripod_A = list(getattr(config, "TRIPOD_A", [0, 2, 4]))
        self.tripod_B = list(getattr(config, "TRIPOD_B", [1, 3, 5]))

        neutral = getattr(config, "FOOT_NEUTRAL_H", None)
        if neutral is None:
            neutral = [(80.0, 0.0, -80.0)] * 6
        if len(neutral) != 6:
            raise RuntimeError("FOOT_NEUTRAL_H must have 6 entries if provided.")
        self.neutral_H = [tuple(p) for p in neutral]

        self.enabled = True
        self.cmd_in = (0.0, 0.0, 0.0)
        self.cmd_latched = (0.0, 0.0, 0.0)
        self.phase = 0
        self.t_phase = 0.0

        self.p_H = [p for p in self.neutral_H]

        self.swing_start = [p for p in self.p_H]
        self.swing_target = [p for p in self.p_H]
        self.swing_h = [self.clearance_base] * 6
        self.stance_start = [p for p in self.p_H]
        self.stance_end = [p for p in self.p_H]

        self._replan_phase(latch_cmd=True)

    def set_enabled(self, enabled: bool):
        self.enabled = bool(enabled)

    def set_command(self, vx, vy, wz=0.0):
        self.cmd_in = (float(vx), float(vy), float(wz))

    def reset_pose(self, neutral=None, send=True):
        if neutral is None:
            neutral = self.neutral_H
        if len(neutral) != 6:
            raise ValueError("neutral must have 6 (x,y,z) tuples.")
        self.p_H = [tuple(p) for p in neutral]
        self.phase = 0
        self.t_phase = 0.0
        self.cmd_latched = (0.0, 0.0, 0.0)
        self._replan_phase(latch_cmd=True)
        if send:
            ik.move_all_legs_xyz(self.p_H, t_ms=300)

    def update(self, dt=None, send=True):
        if dt is None:
            dt = float(getattr(config, "DT", 0.02))
        dt = float(dt)

        if not self.enabled:
            return list(self.p_H)

        vx, vy, wz = self.cmd_in
        if abs(vx) < self.cmd_deadband and abs(vy) < self.cmd_deadband and abs(wz) < 1e-4:
            if send:
                ik.move_all_legs_xyz(self.p_H, t_ms=int(max(10, dt * 1000)))
            return list(self.p_H)

        self.t_phase += dt
        while self.t_phase >= self.phase_T:
            self.t_phase -= self.phase_T
            self.phase = 1 - self.phase
            self._replan_phase(latch_cmd=True)

        u = self.t_phase / self.phase_T

        swing_legs, stance_legs = self._current_groups()

        for i in swing_legs:
            self.p_H[i] = trajectory.swing(
                self.swing_start[i],
                self.swing_target[i],
                u,
                self.swing_h[i],
                style=self.swing_style,
                ease_kind=self.swing_ease,
            )

        for i in stance_legs:
            self.p_H[i] = trajectory.stance(
                self.stance_start[i],
                self.stance_end[i],
                u,
                ease_kind=self.stance_ease,
            )

        if send:
            t_ms = int(max(10, dt * 1000))
            ik.move_all_legs_xyz(self.p_H, t_ms=t_ms)

        return list(self.p_H)

    def _current_groups(self):
        if self.phase == 0:
            return self.tripod_A, self.tripod_B
        return self.tripod_B, self.tripod_A

    def _replan_phase(self, latch_cmd=True):
        if latch_cmd:
            self.cmd_latched = self._clamp_cmd(self.cmd_in)

        vx, vy, wz = self.cmd_latched
        swing_legs, stance_legs = self._current_groups()

        for i in range(6):
            self.swing_start[i] = self.p_H[i]
            self.stance_start[i] = self.p_H[i]

        for i in swing_legs:
            dH = self._phase_delta_hip(i, vx, vy, wz, sign=+1.0)
            self.swing_target[i] = v_add(self.swing_start[i], dH)
            self.swing_h[i] = self._compute_clearance(dH)

        for i in stance_legs:
            dH = self._phase_delta_hip(i, vx, vy, wz, sign=-1.0)
            self.stance_end[i] = v_add(self.stance_start[i], dH)

    def _clamp_cmd(self, cmd):
        vx, vy, wz = float(cmd[0]), float(cmd[1]), float(cmd[2])

        if abs(vx) < self.cmd_deadband:
            vx = 0.0
        if abs(vy) < self.cmd_deadband:
            vy = 0.0

        dx = vx * self.phase_T
        dy = vy * self.phase_T
        step_len = math.hypot(dx, dy)
        if step_len > self.step_limit and step_len > 1e-9:
            scale = self.step_limit / step_len
            dx *= scale
            dy *= scale
            vx = dx / self.phase_T
            vy = dy / self.phase_T

        return (vx, vy, wz)

    def _compute_clearance(self, dH):
        step_len = v_norm2_xy(dH)
        h = self.clearance_base + self.clearance_gain * step_len
        return clamp(h, self.clearance_base, self.clearance_max)

    def _phase_delta_hip(self, leg_idx, vx, vy, wz, sign=+1.0):
        i = int(leg_idx)
        T = self.phase_T

        trans_B = (sign * vx * T, sign * vy * T, 0.0)

        yaw = sign * wz * T
        if abs(yaw) > 1e-8:
            pB = frames.hip_point_to_body(i, self.p_H[i])
            pB_rot = rotz(yaw, pB)
            rot_delta_B = v_sub(pB_rot, pB)
        else:
            rot_delta_B = (0.0, 0.0, 0.0)

        delta_B = v_add(trans_B, rot_delta_B)
        return frames.body_vec_to_hip(i, delta_B)
