# Tabletop Hexapod Control Logic
## 3-Layer Architecture for Relative Walking (No World Frame)

**Goal:** a clean separation between:
1) **Intent (user command)** → 2) **Foot targets & trajectories** → 3) **IK + synchronized servo packets**.

**Target platform:** MicroPython + IK + grouped multi-servo commands.

---

## 0) Coordinate Conventions
Lock these first. Everything assumes they never change.

### Frames
- **Body frame (B):** rigidly attached to the chassis.
  - **Origin: body center (LOCKED).**
- **Hip-local frame (Hᵢ):** rigidly attached to leg *i*.
  - **Origin: hip joint of leg i (0, 0, 0).**

**No world frame required:** during stance, we keep each stance foot “fixed to the ground” by updating a **body-frame anchor** every tick (§2A). This behaves like a world-fixed foot, but expressed entirely in body coordinates.

---

### Axes
We use two axis sets: **Body (B)** and **Hip-local (Hᵢ)**.

#### Body frame axes (B)
Used for: user commands *(v, ω)*, hip positions `hip_pos_B[i]`, and any internal body pose you maintain.
- **+x_B:** forward
- **+y_B:** left
- **+z_B:** up
- Right-handed frame.
- **Yaw sign (LOCKED):**
  - **ω > 0** or **Δψ > 0** = **CCW** viewed from above = **turn left**.
- Command meaning:
  - `v_x` = forward speed (mm/s)
  - `v_y` = leftward speed (mm/s)

#### Hip-local axes (Hᵢ)
Used for IK inputs: foot targets `(x, y, z)` passed into `IK(x, y, z)`.
- **Origin:** hip joint of leg i `(0, 0, 0)`
- **z:** height
  - **z < 0** means the foot is below the hip (down)
- **x:** radial direction at this hip
  - Direction is defined by **body center → hip → outward**
  - Important: the hip is the origin; “body center → hip” defines *direction only*.
  - **x > 0** = outward (away from body center)
  - **x < 0** = inward (toward body center)
- **y:** tangential direction (completes a right-handed frame)
  - Looking down from above: **+y is CCW tangent**
- **Coxa yaw reference:**
  - `coxa = atan2(y, x)`
  - `coxa = 0°` points straight outward along **+x**

---

### Required Fixed Data
These are constants/config values you set once (or calibrate).
- `hip_pos_B[i] = (x, y, z)` for i = 0..5 (hip locations in body frame)
- `hip_yaw0[i]` for i = 0..5 (mount yaw that aligns the body-frame radial direction to hip-local +x)
  - Common definition (body origin at center):
    - `hip_yaw0[i] = atan2(hip_pos_B[i].y, hip_pos_B[i].x)`
- Link lengths: `COXA_L`, `FEMUR_L`, `TIBIA_L`

---

## 1) Layer 1 — Intent → Gait Targets (High-Level Controller)

### Inputs
- Body-frame velocity command:
  - `v_B = (v_x, v_y)` in mm/s
  - `ω` yaw rate in rad/s
- Optional body height setpoint: `z_body` (mm)

### Outputs (to Layer 2)
For each leg i:
- `phase[i] ∈ {STANCE, SWING}`
- Timing parameters:
  - cycle period `T`
  - duty factor `β` (stance ratio)
- Swing endpoints in **body frame**:
  - `p0_B[i]` = lift-off point (captured at swing start)
  - `p1_B[i]` = planned touchdown point

### State (body-relative)
- Normalized gait phase time: `t_phase ∈ [0, 1)`
- Stance anchor in body frame: `p_anchor_B[i]`
  - Set at touch-down
  - Updated each tick during stance so it remains ground-fixed in a body-relative sense

---

## 2) Layer 2 — Foot Trajectory Generator (Swing + Stance)
This layer is pure geometry over time, entirely in **body coordinates**.

### Inputs
- `phase[i]` and normalized swing time `s ∈ [0, 1]` (only meaningful for swing legs)
- Commanded body motion per tick:
  - `Δp_B = v_B · dt`
  - `Δψ = ω · dt`
- Clearance height: `h_clear`
- Swing endpoints: `p0_B[i]`, `p1_B[i]`

### Outputs
- `p_des_B[i](t)`: desired foot position in body frame

---

### 2A) Stance Trajectory (Ground Contact, No World Frame)
During stance we want the foot to be **fixed on the ground** while the body moves.

We maintain a stance anchor in body coordinates, `p_anchor_B[i]`, and update it every tick using the *commanded* body motion.

#### Per-tick body motion
Given `(v_x, v_y, ω)` and loop time step `dt`:
- Translation in body frame:
  - `Δx = v_x · dt`
  - `Δy = v_y · dt`
  - `Δp_xy = [Δx, Δy]^T`
- Yaw increment:
  - `Δψ = ω · dt`

#### Rotation matrix (2D yaw)
For yaw angle θ, the 2D rotation in the body XY plane is:
- `R(θ) = [[cosθ, −sinθ], [sinθ, cosθ]]`

#### Anchor update (ground-fixed, expressed in body coordinates)
For each leg i in **STANCE**, update the XY part of the anchor:
- Let `a_xy` be the anchor XY vector.
- Update rule:
  - `a_xy ← R(−Δψ) · (a_xy − Δp_xy)`
- Keep stance height constant:
  - `a_z ← z0` (nominal stance height, typically `z0 = −z_body`)

Then the stance desired foot position is:
- `p_des_B[i] = p_anchor_B[i]`

**Simplified first pass (no turning):** if `ω = 0`, then `Δψ = 0` and:
- `a_xy ← a_xy − Δp_xy`

---

### 2B) Swing Trajectory (Air)
Define start/end in **body frame**:
- Start: `p0_B[i]` (lift-off)
- End:   `p1_B[i]` (touch-down)

Swing duration:
- `T_swing = (1 − β) · T`

Normalized swing time:
- `s = clamp((t − t_liftoff) / T_swing, 0, 1)`

#### Smooth horizontal interpolation (zero velocity at both ends)
Use a cubic smoothstep:
- `u(s) = 3s^2 − 2s^3`

Then horizontal (XY) position:
- `p_xy(s) = (1 − u) · p0_xy + u · p1_xy`

#### Smooth vertical lift (zero velocity at both ends)
Let `z0` be nominal stance height (usually `z0 = −z_body`) and `h_clear > 0` be clearance.

Use a 4th-order “bump” function (0 at ends, 1 at mid-swing):
- `b(s) = 16 s^2 (1 − s)^2`

Vertical position:
- `z(s) = z0 + h_clear · b(s)`

Combined swing target:
- `p_des_B(s) = (p_xy(s), z(s))`

---

### 2C) Terrain Awareness (TBC)
Not in scope for the “get it moving” milestone.

---

## 3) Layer 3 — Kinematics + Synchronized Servo Output

### Inputs
- `p_des_B[i]`: desired foot position in body frame

### Outputs
- One synchronized command packet containing 18 servo targets

---

### 3A) Transform Body → Hip-local
For each leg i:
1) Translate to the hip origin (body frame):
   - `p_rel_B = p_des_B − hip_pos_B[i]`
2) Rotate into hip-local (mount yaw):
   - `p_H = Rz(−hip_yaw0[i]) · p_rel_B`

### 3B) Solve IK (per leg)
- `(coxa, femur, tibia) = IK(p_H.x, p_H.y, p_H.z)`

### 3C) Map logical angles → servo angles
- `servo_deg = 90 + sign · logical + offset`

### 3D) Send one grouped packet
- Build: `pairs = [(channel, servo_deg), ...]` for all 18 servos
- Send once: `set_pose(pairs, t_ms)`

---

## 4) Logic Loop (What Runs Every Tick)
Run this at `dt = 10–20 ms` (50–100 Hz).

1) **Read input** → `(v_x, v_y, ω)`
2) Compute per-tick deltas:
   - `Δx = v_x · dt`, `Δy = v_y · dt`, `Δψ = ω · dt`
3) **Layer 1:** update gait phase(s), and handle transitions:
   - On **STANCE → SWING** (lift-off): latch `p0_B[i] = p_anchor_B[i]`, plan `p1_B[i]`, set `s=0`
   - On **SWING → STANCE** (touch-down): set `p_anchor_B[i] = p1_B[i]`
4) **Layer 2A:** for legs in stance, update `p_anchor_B` using the anchor update rule
5) **Layer 2:** compute `p_des_B[i]`:
   - STANCE: `p_des_B = p_anchor_B`
   - SWING: `p_des_B = swing_pos(p0_B, p1_B, s)`
6) **Transforms:** convert each `p_des_B[i]` → hip-local `p_H[i]`
7) **Layer 3:** IK + build 18-servo packet
8) Send one packet: `set_pose(pairs, t_ms)`

---

## 5) Required File Structure (MVP)
This is the minimal set to stand, swing, and start walking.

```
hexapod/
  main.py                # main loop (glue)
  config.py              # geometry + calibration + mappings
  servo_uart.py          # UART packet output (group moves)
  ik_fixed.py            # hip-local IK + move_all_legs_xyz()
  frames.py              # body → hip-local transform
  trajectory.py          # stance_pos() and swing_pos()
  gait_tripod.py         # tripod schedule + p0/p1 planning
```

Notes:
- You already have `servo_uart.py` and `ik_fixed.py` in this project.
- Below are **MVP templates** for the missing files, plus the key functions you need.

---

## 6) Required Python Files (MVP Templates)

### `config.py` (constants + mapping)
```python
# config.py

# Link lengths (mm)
COXA_L  = 0.0
FEMUR_L = 0.0
TIBIA_L = 0.0

# Leg servo channels: (coxa, femur, tibia) for legs 0..5
LEG_SERVOS = [
    (0, 1, 2),
    (3, 4, 5),
    (6, 7, 8),
    (9, 10, 11),
    (12, 13, 14),
    (15, 16, 17),
]

# Per-channel direction and offset
SERVO_INVERT = [False] * 24
SERVO_OFFSET = [0.0] * 24

# Hip positions in BODY frame (mm), origin at body center
hip_pos_B = [
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
]

# Mount yaw per hip (rad). Common: atan2(y, x) from hip_pos_B
hip_yaw0 = [0.0] * 6
```

---

### `frames.py` (body → hip-local transform)
```python
# frames.py
import math
import config

def rotz(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return c, s

def b_to_hi(p_B, i):
    """Body-frame point -> hip-local point for leg i."""
    x, y, z = p_B
    hx, hy, hz = config.hip_pos_B[i]

    # translate to hip origin
    xr = x - hx
    yr = y - hy
    zr = z - hz

    # rotate into hip-local
    th = -config.hip_yaw0[i]
    c, s = rotz(th)
    xh = c * xr - s * yr
    yh = s * xr + c * yr
    zh = zr
    return (xh, yh, zh)
```

---

### `trajectory.py` (stance + swing math)
```python
# trajectory.py

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def stance_pos(anchor_B):
    return anchor_B

def smoothstep(s):
    # u(s) = 3s^2 - 2s^3
    return 3*s*s - 2*s*s*s

def bump(s):
    # b(s) = 16 s^2 (1-s)^2
    t = 1 - s
    return 16 * s*s * t*t

def swing_pos(p0_B, p1_B, s, z0, h_clear):
    s = clamp(s, 0.0, 1.0)
    u = smoothstep(s)

    x0, y0, _ = p0_B
    x1, y1, _ = p1_B

    x = (1-u)*x0 + u*x1
    y = (1-u)*y0 + u*y1

    z = z0 + h_clear * bump(s)
    return (x, y, z)
```

---

### `gait_tripod.py` (tripod schedule + p0/p1 planning)
```python
# gait_tripod.py

# You must decide your leg indices.
# Example groups (edit to match your mapping):
GROUP_A = [0, 3, 4]
GROUP_B = [1, 2, 5]

STANCE = 0
SWING  = 1

class TripodGait:
    def __init__(self, T=1.0, beta=0.6):
        self.T = float(T)
        self.beta = float(beta)
        self.t = 0.0

        self.phase = [STANCE]*6
        self.s = [0.0]*6

        self.p_anchor_B = [(0,0,0)]*6
        self.p0_B = [(0,0,0)]*6
        self.p1_B = [(0,0,0)]*6

    def init_anchors(self, anchors_B):
        self.p_anchor_B = list(anchors_B)

    def plan_next_anchor(self, i, v_x, v_y, step_len):
        # MVP: move along commanded direction with fixed step length.
        # Replace later with better planning.
        mag = (v_x*v_x + v_y*v_y) ** 0.5
        if mag < 1e-6:
            return self.p_anchor_B[i]
        dx = (v_x/mag) * step_len
        dy = (v_y/mag) * step_len
        ax, ay, az = self.p_anchor_B[i]
        return (ax + dx, ay + dy, az)

    def update(self, dt, v_x, v_y, omega, step_len):
        self.t = (self.t + dt) % self.T

        # Simple half-cycle toggling
        half = 0 if self.t < self.T/2 else 1
        swing_group = GROUP_A if half == 0 else GROUP_B

        # Set phases
        for i in range(6):
            self.phase[i] = SWING if i in swing_group else STANCE

        # Update swing progress for swing legs
        T_swing = (1.0 - self.beta) * self.T
        if T_swing < 1e-6:
            T_swing = 1e-6

        # We latch p0/p1 at the moment a leg enters swing.
        # MVP: detect “enter swing” by s==0 and phase==SWING at start of half.
        # You can refine this later.
        for i in range(6):
            if self.phase[i] == SWING:
                # re-latch at the start of each half-cycle
                if abs((self.t % (self.T/2))) < dt:
                    self.p0_B[i] = self.p_anchor_B[i]
                    self.p1_B[i] = self.plan_next_anchor(i, v_x, v_y, step_len)
                    self.s[i] = 0.0
                else:
                    self.s[i] = min(1.0, self.s[i] + dt / T_swing)
            else:
                self.s[i] = 0.0

        return self.phase, self.s, self.p0_B, self.p1_B
```

---

### `main.py` (glue loop)
```python
# main.py
import time
import config
import frames
import trajectory
import ik_fixed

from gait_tripod import TripodGait, STANCE, SWING

# --- Tunables ---
dt_s = 0.02          # 20 ms tick
z_body = 80.0        # mm (pick a safe height)
z0 = -z_body
h_clear = 15.0       # mm
step_len = 15.0      # mm (small first)
T = 1.0
beta = 0.6

# Neutral stance anchors in BODY frame (you must fill these)
anchors_B = [
    (0.0, 0.0, z0),
    (0.0, 0.0, z0),
    (0.0, 0.0, z0),
    (0.0, 0.0, z0),
    (0.0, 0.0, z0),
    (0.0, 0.0, z0),
]

# Example command (replace with joystick/input)
v_x = 30.0   # forward mm/s
v_y = 0.0
omega = 0.0

gait = TripodGait(T=T, beta=beta)
gait.init_anchors(anchors_B)

while True:
    # 1) commanded deltas
    dx = v_x * dt_s
    dy = v_y * dt_s
    dpsi = omega * dt_s

    # 2) gait update (Layer 1)
    phase, s_list, p0_list, p1_list = gait.update(dt_s, v_x, v_y, omega, step_len)

    # 3) update stance anchors (Layer 2A)
    for i in range(6):
        if phase[i] == STANCE:
            ax, ay, _ = gait.p_anchor_B[i]
            # simplified: no yaw compensation until omega is stable
            gait.p_anchor_B[i] = (ax - dx, ay - dy, z0)

    # 4) compute p_des_B (Layer 2)
    p_des_B = [None]*6
    for i in range(6):
        if phase[i] == STANCE:
            p_des_B[i] = trajectory.stance_pos(gait.p_anchor_B[i])
        else:
            p_des_B[i] = trajectory.swing_pos(p0_list[i], p1_list[i], s_list[i], z0, h_clear)

    # 5) body -> hip-local targets (Layer 3A)
    targets_H = [frames.b_to_hi(p_des_B[i], i) for i in range(6)]

    # 6) IK + send one packet (Layer 3)
    ik_fixed.move_all_legs_xyz(targets_H, t_ms=int(dt_s*1000))

    time.sleep(dt_s)
```

---

## 7) What You Must Fill In First
To make the MVP move, you must set:
1) `config.hip_pos_B[i]` and `config.hip_yaw0[i]`
2) Neutral stance anchors `anchors_B[i]` (reasonable x/y spreads + `z0`)
3) Tripod groups `GROUP_A/GROUP_B` to match your leg indices
4) Basic tunables: `z_body`, `h_clear`, `step_len`, `T`, `beta`

