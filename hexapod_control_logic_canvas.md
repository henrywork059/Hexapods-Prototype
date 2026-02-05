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
  - This is the only global frame required for the MVP.
- **Hip-local frame (Hᵢ):** rigidly attached to leg *i*.
  - **Origin: the hip joint of leg i (0, 0, 0).**

**No world frame required:** we do not rely on external localization. During stance, we keep each stance foot “fixed to the ground” by updating a **body-frame anchor** every tick (§2A). This behaves like a world-fixed foot, but expressed entirely in body coordinates.

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

### Core decisions
- Gait type: tripod / ripple / wave
- Leg grouping and swing schedule
- Transition rules: lift-off, touch-down, and phase switching

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

We achieve this by maintaining a stance anchor in body coordinates, `p_anchor_B[i]`, and updating it every tick using the *commanded* body motion.

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

This avoids sharp velocity jumps at lift-off and touch-down.

#### Smooth vertical lift (zero velocity at both ends)
Let `z0` be nominal stance height (usually `z0 = −z_body`) and `h_clear > 0` be clearance.

Use a 4th-order “bump” function (0 at ends, 1 at mid-swing):
- `b(s) = 16 s^2 (1 − s)^2`

Vertical position:
- `z(s) = z0 + h_clear · b(s)`

Properties:
- `z(0) = z0`, `z(1) = z0`
- `dz/dt = 0` at lift-off and touch-down
- `z(0.5) = z0 + h_clear` (peak clearance)

#### Combined swing target
- `p_des_B(s) = (p_xy(s), z(s))`

Notes:
- Because `+z_B` is up, increasing `z` (less negative) lifts the foot.
- Keep `h_clear` small at first (e.g., 10–20 mm) until motion is stable.

---

### 2C) Terrain Awareness (TBC)
Not in scope for the “get it moving” milestone.

For now:
- Use fixed timing for swing (no early contact termination).
- Use conservative clearance `h_clear` and a safe nominal stance height.

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

## 4) Suggested Update Loop (Single Thread)

### Timing
- Control tick: `dt = 10–20 ms` (50–100 Hz)

### Loop (relative walking)
1) Read user input → `(v_B, ω)`
2) Compute per-tick motion: `Δp_B = v_B·dt`, `Δψ = ω·dt`
3) Update gait state (Layer 1): phases + swing timing + endpoints
4) Update stance anchors for all stance legs:
   - `p_anchor_B ← Rz(−Δψ)·(p_anchor_B − Δp_B)`
5) For each leg:
   - STANCE: `p_des_B = p_anchor_B`
   - SWING:  `p_des_B = swing_pos(p0_B, p1_B, s, h_clear)`
6) Transform `p_des_B → p_H` (hip-local)
7) IK → servo targets
8) Send one grouped packet

---

## 6) Suggested Code Modules and Key Functions

### Layer 0 — Conventions + Config
**config.py**
- constants: `COXA_L, FEMUR_L, TIBIA_L`
- servo mapping: `LEG_SERVOS[6] = (coxa_ch, femur_ch, tibia_ch)`
- calibration: `SERVO_INVERT[ch]`, `SERVO_OFFSET[ch]`
- hip geometry: `hip_pos_B[6]`, `hip_yaw0[6]`

### Layer 3 — Kinematics + Servo IO
**servo_uart.py**
- `set_pose(pairs, t_ms)` → send many servos in one packet
- `set_servo(ch, angle_deg, t_ms)` → single servo (debug)

**ik_fixed.py** (expects hip-local Hᵢ targets)
- `ik_xyz(x, y, z)` → logical joint angles
- `move_leg_xyz(leg_idx, x, y, z, t_ms)` → 3-servo packet for one leg
- `move_all_legs_xyz(targets6, t_ms)` → 18-servo packet for all legs

**frames.py** (body → hip-local transforms)
- `b_to_hi(p_B, i)`
- `b_to_hi_vec(dxdy_B, i)` (optional helper for small deltas)

### Layer 2 — Foot Trajectories
**trajectory.py**
- `stance_pos(anchor_B)` → `anchor_B`
- `swing_pos(p0_B, p1_B, s, h_clear)` → smooth swing curve in body frame
- optional: `touchdown_filter(p_des_B, contact)`

### Layer 1 — Gait + Step Target Planning
**gait_tripod.py**
- `init_gait(default_anchors_B)`
- `update_gait(dt, v_B, omega)`
  - outputs per leg: `phase[i]`, `s[i]`, `p0_B[i]`, `p1_B[i]`
- `plan_next_anchor(i, v_B, omega, step_len)` → returns `p1_B[i]`

### Glue / Main Loop
**controller.py / main.py**
- read user input → call Layer 1
- Layer 2 generates `p_des_B[i]`
- transform body → hip-local
- call `ik_fixed.move_all_legs_xyz(targets_H, t_ms)`

