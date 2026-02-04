\
# gait_tripod.py
# Tripod gait scheduler with:
#   - input latching (commands only take effect at phase boundaries)
#   - phase-boundary replanning (no mid-swing retarget unless you add reflex logic)
#   - stance + swing trajectories from trajectory.py
#   - hip-local IK via ik.py (or ik_fixed.py fallback)
#
# This version is designed for your CURRENT setup:
#   - No external "world frame" required
#   - Works purely with body frame + hip frames (relative coordinates)
#
# Frames used:
#   - Body frame B: origin at body center, +x forward, +y left, +z up
#   - Hip frame H_i: origin at hip, +x radial outward, +y left, +z up
#
# Foot positions passed into IK are hip-local (H_i).
#
# How it "moves" without world coords:
#   - During STANCE: we move the stance feet backward in hip frame (relative to body) by a
#     planned delta over the phase (as if body is moving forward over the ground).
#   - During SWING: we move the swing feet forward by the opposite delta and lift in z.
#
# If you later add a world frame, this file can be upgraded easily, but it already works
# for flat-ground open-loop walking tests.

import math
import time

# --- config import (supports either config.py or config_pca.py) ---
try:
    import config
except ImportError:
    import config_pca as config  # fallback if you didn't rename config_pca.py

import frames
import trajectory

# --- IK import (supports either ik.py or ik_fixed.py) ---
try:
    import ik as ik
except ImportError:
    import ik_fixed as ik


def _sleep_ms(ms: int):
    try:
        time.sleep_ms(int(ms))
    except AttributeError:
        time.sleep(float(ms) / 1000.0)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_norm2_xy(v):
    return math.hypot(v[0], v[1])


def rotz(theta, p):
    """Rotate a point/vector in XY plane by theta around origin."""
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
        # -----------------------------
        # User-tunable parameters (with safe defaults)
        # -----------------------------
        self.period = float(getattr(config, "GAIT_PERIOD", 0.8))  # seconds per full gait cycle
        self.phase_T = 0.5 * self.period                          # half-cycle duration

        self.swing_style = str(getattr(config, "SWING_STYLE", "sine"))
        self.swing_ease = str(getattr(config, "SWING_EASE", "quintic"))
        self.stance_ease = str(getattr(config, "STANCE_EASE", "linear"))

        # Clearance (mm): base + gain * step_length
        self.clearance_base = float(getattr(config, "SWING_CLEARANCE", 25.0))
        self.clearance_gain = float(getattr(config, "SWING_CLEARANCE_GAIN", 0.15))
        self.clearance_max = float(getattr(config, "SWING_CLEARANCE_MAX", 50.0))

        # Stop threshold (commands smaller than this are treated as zero)
        self.cmd_deadband = float(getattr(config, "CMD_DEADBAND", 0.5))  # mm/s equiv

        # Optional: limit step length per phase (mm) to stay safe
        self.step_limit = float(getattr(config, "STEP_LIMIT_MM", 40.0))

        # Tripod leg groups (default for leg order: 0 FR,1 MR,2 BR,3 FL,4 ML,5 BL)
        self.tripod_A = list(getattr(config, "TRIPOD_A", [0, 2, 4]))
        self.tripod_B = list(getattr(config, "TRIPOD_B", [1, 3, 5]))

        # Neutral foot pose in HIP frame (mm).
        # You can override in config as FOOT_NEUTRAL_H = [(x,y,z),...x6]
        neutral = getattr(config, "FOOT_NEUTRAL_H", None)
        if neutral is None:
            # Safe-ish placeholder: forward and down
            neutral = [(80.0, 0.0, -80.0)] * 6
        if len(neutral) != 6:
            raise RuntimeError("FOOT_NEUTRAL_H must have 6 entries if provided.")
        self.neutral_H = [tuple(p) for p in neutral]

        # -----------------------------
        # Internal state
        # -----------------------------
        self.enabled = True

        self.cmd_in = (0.0, 0.0, 0.0)       # latest command (not latched)
        self.cmd_latched = (0.0, 0.0, 0.0)  # used for current phase

        self.phase = 0          # 0 or 1
        self.t_phase = 0.0      # seconds within current half-cycle

        # Current desired feet positions in hip frames
        self.p_H = [p for p in self.neutral_H]

        # Per-leg plans for current phase
        self.swing_start = [p for p in self.p_H]
        self.swing_target = [p for p in self.p_H]
        self.swing_h = [self.clearance_base] * 6

        self.stance_start = [p for p in self.p_H]
        self.stance_end = [p for p in self.p_H]

        # Plan initial phase
        self._replan_phase(latch_cmd=True)

    # -----------------------------
    # External API
    # -----------------------------
    def set_enabled(self, enabled: bool):
        self.enabled = bool(enabled)

    def set_command(self, vx, vy, wz=0.0):
        """Set the (unlatched) body command in BODY frame.

        vx, vy in mm/s; wz in rad/s.
        """
        self.cmd_in = (float(vx), float(vy), float(wz))

    def reset_pose(self, neutral=None, send=True):
        """Reset internal foot state to neutral pose (or provided pose) and replan.

        neutral: optional list of 6 hip-local foot positions.
        send: if True, sends to IK immediately.
        """
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
        """Advance gait by dt seconds and optionally send targets to IK.

        Returns: list of 6 hip-local foot targets.
        """
        if dt is None:
            dt = float(getattr(config, "DT", 0.02))
        dt = float(dt)

        if not self.enabled:
            return list(self.p_H)

        # If command is essentially zero, hold pose and do not advance gait clock.
        vx, vy, wz = self.cmd_in
        if abs(vx) < self.cmd_deadband and abs(vy) < self.cmd_deadband and abs(wz) < 1e-4:
            # Slowly relax to neutral if you want, but for now just hold current.
            if send:
                ik.move_all_legs_xyz(self.p_H, t_ms=int(max(10, dt * 1000)))
            return list(self.p_H)

        # Advance time
        self.t_phase += dt

        # Handle phase wrap (may wrap multiple times if dt is huge)
        while self.t_phase >= self.phase_T:
            self.t_phase -= self.phase_T
            self.phase = 1 - self.phase
            self._replan_phase(latch_cmd=True)

        u = self.t_phase / self.phase_T  # 0..1

        # Compute current targets from plan
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
            # Drive time is dt (ms). If your servo driver blocks too long, set t_ms=None.
            t_ms = int(max(10, dt * 1000))
            ik.move_all_legs_xyz(self.p_H, t_ms=t_ms)

        return list(self.p_H)

    # -----------------------------
    # Planning
    # -----------------------------
    def _current_groups(self):
        """Return (swing_legs, stance_legs) for current phase."""
        if self.phase == 0:
            # A swings, B stances
            return self.tripod_A, self.tripod_B
        else:
            # B swings, A stances
            return self.tripod_B, self.tripod_A

    def _replan_phase(self, latch_cmd=True):
        """Replan swing/stance start/end at phase boundary (no mid-swing retarget)."""
        if latch_cmd:
            self.cmd_latched = self._clamp_cmd(self.cmd_in)

        vx, vy, wz = self.cmd_latched

        swing_legs, stance_legs = self._current_groups()

        # Start points are ALWAYS the current p_H at the boundary
        for i in range(6):
            self.swing_start[i] = self.p_H[i]
            self.stance_start[i] = self.p_H[i]

        # Plan deltas per leg
        for i in swing_legs:
            dH = self._phase_delta_hip(i, vx, vy, wz, sign=+1.0)
            self.swing_target[i] = v_add(self.swing_start[i], dH)
            self.swing_h[i] = self._compute_clearance(dH)

        for i in stance_legs:
            dH = self._phase_delta_hip(i, vx, vy, wz, sign=-1.0)
            self.stance_end[i] = v_add(self.stance_start[i], dH)

    def _clamp_cmd(self, cmd):
        """Apply deadband and optional step limit."""
        vx, vy, wz = float(cmd[0]), float(cmd[1]), float(cmd[2])

        # Deadband on translation
        if abs(vx) < self.cmd_deadband:
            vx = 0.0
        if abs(vy) < self.cmd_deadband:
            vy = 0.0

        # Convert desired translation over phase into step length and clamp
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
        """Compute swing lift height (mm) based on horizontal step length."""
        step_len = v_norm2_xy(dH)
        h = self.clearance_base + self.clearance_gain * step_len
        return clamp(h, self.clearance_base, self.clearance_max)

    def _phase_delta_hip(self, leg_idx, vx, vy, wz, sign=+1.0):
        """Compute planned delta in HIP frame over one half-cycle.

        sign = +1 for SWING planning (foot moves forward relative body)
        sign = -1 for STANCE planning (foot moves backward relative body)

        Includes:
          - translation in BODY: (vx, vy) * phase_T
          - yaw effect (optional): rotate current foot point in BODY about origin
        """
        i = int(leg_idx)
        T = self.phase_T

        # Translation part in BODY frame
        trans_B = (sign * vx * T, sign * vy * T, 0.0)

        # Yaw part: rotate the current foot point in BODY frame
        yaw = sign * wz * T
        if abs(yaw) > 1e-8:
            # Use current foot position expressed in BODY frame to compute rotation delta
            pB = frames.hip_point_to_body(i, self.p_H[i])
            pB_rot = rotz(yaw, pB)
            rot_delta_B = v_sub(pB_rot, pB)
        else:
            rot_delta_B = (0.0, 0.0, 0.0)

        delta_B = v_add(trans_B, rot_delta_B)

        # Convert delta BODY vector into HIP frame vector
        delta_H = frames.body_vec_to_hip(i, delta_B)
        return delta_H


# ------------------------------------------------------------
# Simple demo (for MicroPython):
# ------------------------------------------------------------
if __name__ == "__main__":
    gait = TripodGait()
    gait.reset_pose(send=True)
    _sleep_ms(800)

    # Walk forward slowly for a few seconds (open loop)
    gait.set_command(40.0, 0.0, 0.0)  # vx=40mm/s
    t0 = time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)

    while True:
        gait.update()
        _sleep_ms(int(getattr(config, "DT", 0.02) * 1000))

        now = time.ticks_ms() if hasattr(time, "ticks_ms") else int(time.time() * 1000)
        if (now - t0) > 5000:
            break

    # Stop
    gait.set_command(0.0, 0.0, 0.0)
    gait.update(send=True)
    print("Demo done.")
