# Hexapod Project: Required Code Files (Build + Validation Order)

This is the **complete MVP + validation** file list to get your hexapod moving with your current approach:
- **No world frame** (for now)
- **Body origin at body center**
- **Hip-local IK** per leg
- **Latched swing targets** (plan at phase boundary; no mid-swing retarget)
- **One grouped packet** to move all 18 servos per control tick

---

## 0) Configuration foundation

### 1) `config.py` ✅ *(develop first)*
**Purpose:** single source of truth for geometry + servo mapping + limits + timing.

**Must include:**
- UART: `UART_ID, UART_BAUD, UART_TX, UART_RX, UART_SPACING_MS`
- Link lengths: `COXA_L, FEMUR_L, TIBIA_L`
- Servo mapping: `LEG_SERVOS[6] = [(coxa,femur,tibia), ...]`
- Direction/offsets: `SERVO_INVERT[24]`, `SERVO_OFFSET[24]`
- (Recommended) per-servo clamp: `SERVO_MIN_A[24]`, `SERVO_MAX_A[24]`
- Body geometry: `hip_pos_B[6]`, `hip_yaw0[6]`
- Timing defaults: `SERVO_MOVE_MS`, control `DT`

**Validate:** import succeeds; print/inspect mappings and lengths.

---

## 1) Layer 3: Hardware IO (servo packets)

### 2) `servo_uart.py` ✅
**Purpose:** send **single** and **grouped** packets to the YH-24 controller.

**Validate with:**

### 3) `test_uart_center.py`
- `center_all()` using one grouped packet
- move one channel slowly: 70° → 90° → 110°

**Pass criteria:** correct channel moves, no jitter, timing behaves.

---

## 2) Layer 3: IK (hip-local) + packet output

### 4) `ik_fixed.py` ✅
**Purpose:** hip-local `(x,y,z)` → `(coxa,femur,tibia)` logical angles → servo angles → packet.

**Validate with:**

### 5) `test_ik_single_leg_sweep.py`
- pick one leg
- sweep a small grid, e.g.
  - `x: 60..90`, `y: -20..20`, `z: -60..-100`

### 6) `test_ik_all_legs_same_pose.py`
- send same stance pose to all 6 legs (each in its hip-local frame)
- confirm **all 18** servos move together via **one grouped packet**

**Pass criteria:** no leg flips, no wrong joint, knees behave like `/\` not `\/`.

---

## 3) Calibration helpers (highly recommended)

### 7) `calibrate_servos.py`
**Purpose:** tune:
- `SERVO_OFFSET[ch]`
- `SERVO_INVERT[ch]`
- optional min/max clamps

**Validate:** can re-center and achieve a stable neutral stance.

---

## 4) Frames: Body → hip-local transform

### 8) `frames.py` ✅
**Purpose:** convert body-frame foot targets to hip-local targets for IK.

**Key function:**
- `body_to_hip(i, p_B) -> p_H`
  - `p_rel_B = p_B - hip_pos_B[i]`
  - `p_H = Rz(-hip_yaw0[i]) @ p_rel_B`

**Validate with:**

### 9) `test_frames_sanity.py`
- place a small “ring” of body-frame targets around each hip
- verify mapping matches your axes:
  - hip-local +x = outward radial
  - hip-local y = tangent

---

## 5) Layer 2: Trajectory generator (stance + swing)

### 10) `trajectory.py` ✅
**Purpose:** generate smooth foot motion for stance and swing.

**Functions:**
- `update_stance_anchor(p_anchor_B, Δp_B, Δψ)`
- `swing_pos(p0_B, p1_B, s, z0, h_clear)`
  - smooth start/end velocity (e.g. smoothstep)
  - z uses a bump for clearance

**Validate with:**

### 11) `test_trajectory_one_leg_no_ik.py`
- print/log `(x,y,z)` over time
- check:
  - swing starts/ends with ~zero velocity
  - z rises to `z0 + h_clear` then returns

---

## 6) Layer 1: Tripod gait scheduler + latching

### 12) `gait_tripod.py` ✅
**Purpose:** state machine for tripod gait and latching rules.

**Implements:**
- tripod groups A/B
- phase timing
- latch `p0_B` at lift-off
- plan `p1_B` only at phase boundary
- set stance anchor at touchdown

**Validate with:**

### 13) `test_gait_dryrun.py`
No servos—just print:
- which legs are SWING/STANCE each tick
- `p0_B` / `p1_B`
- swing parameter `s: 0→1`

**Pass criteria:** no mid-swing retarget; changes only at phase boundary.

---

## 7) Integration: main control loop

### 14) `main.py` ✅
**Purpose:** your real-time loop at fixed `dt`.

**Tick order:**
1. read input: `v_x, v_y, ω`
2. compute `Δp_B, Δψ`
3. Layer 1 transitions + latching
4. Layer 2 stance update + swing positions
5. frames: `p_des_B -> p_H`
6. IK all legs
7. send **one 18-servo packet**

**Validate with:**

### 15) `walk_smoke_test.py`
- hardcode a tiny forward command
- confirm repeatable stepping without drifting into impossible poses

---

## 8) Input + safety (small but important)

### 16) `input_cmd.py` *(optional but practical)*
- joystick / serial / keyboard → filtered `v_x, v_y, ω`

### 17) `safety.py`
- emergency stop / “sit down now”
- clamp foot targets before IK
- rate limits (avoid position jumps)

---

# Minimum runtime files (MVP)
If you only want what **must run on the robot**:
- `config.py`
- `servo_uart.py`
- `ik_fixed.py`
- `frames.py`
- `trajectory.py`
- `gait_tripod.py`
- `main.py`

Everything else is for **validation/calibration** and will save debugging time.

