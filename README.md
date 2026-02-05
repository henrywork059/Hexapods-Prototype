# Tabletop Hexapod Walking Robot — Hexapods-Prototype

A prototype control stack for a **tabletop hexapod (18 servos, 3DOF/leg)** focused on a clean, testable walking architecture:

1) **Intent (user command)** →  
2) **Foot targets + trajectories** →  
3) **IK + synchronized servo output**

The current MVP goal is **repeatable flat-ground walking in open loop** using **relative coordinates** (no external localization / no required world frame).

---

## Current Progress (What’s Done vs Next)

### Implemented (working pieces are in repo)
- ✅ **3-layer architecture + math spec** (body frame + hip-local frames, stance anchors, latched swing planning)
- ✅ **Tripod gait scheduler** with **phase-boundary command latching** (no mid-swing retarget)
- ✅ **Trajectory generator** (stance interpolation + smooth swing lift: sine/parabolic/bezier)
- ✅ **Hip-local IK** (knee-up branch so legs look like `/\`, not `\/`)
- ✅ **Servo output drivers**
  - **PCA9685 (I2C PWM)** driver (software-interpolated timing)
  - **UART multi-servo packet** driver (legacy / alternative path)
- ✅ **Simulation tools**
  - Python simulation loop + collision/clearance checks
  - HTML 2D gait + latched swing visualizer
  - HTML planner UI (foot targets + stability visualization)

### In Progress / Next to Validate on Hardware
- 🔶 **Servo calibration** (invert, offsets, per-channel limits) to lock a stable neutral stance
- 🔶 **Bring-up tests** (center, single-leg sweep, all-legs same pose, packet timing)
- 🔶 **Folder/config cleanup** (reduce duplicate configs, unify “one source of truth”)
- 🔶 **Input layer** (BLE/joystick filtering → clean `(v_x, v_y, ω)` commands)
- ⏳ **Terrain/IMU adaptation** (explicitly out of scope for MVP; revisit after basic walking)

---

## Current Assumed Settings (MVP Defaults)

These are the conventions the code and docs assume **right now**. If you change them, change them everywhere.

### Coordinate Frames (LOCKED)
**Body frame (B)**
- Origin: **body center**
- Axes: **+x forward, +y left, +z up**
- Yaw sign: **positive = CCW (turn left)** when viewed from above

**Hip-local frame (Hᵢ)** (per leg)
- Origin: **hip joint**
- Axes:
  - **+x** radial **outward** from body center through the hip
  - **+y** tangential (CCW direction)
  - **+z up**  → so **foot below hip is negative z**

### Gait/Timing (typical defaults)
- Control tick: `DT ≈ 0.02s` (50 Hz)
- Tripod half-cycle: `phase_T = period/2` (default `period ≈ 0.8s` unless overridden)
- Command policy: **latch command at phase boundary**, replan swing endpoints only at boundary
- Step limiting: optional clamp (default `STEP_LIMIT_MM ≈ 40mm` in the tripod controller)

### Hardware Assumptions (current direction)
**Primary target:** MicroPython MCU (ex: RP2040/Pico-class) driving **PCA9685 over I2C**
- PCA9685 PWM frequency: **50 Hz**
- Typical pulse range: **500–2500 µs**
- 18 servos → usually **2 PCA boards** (example addresses: `0x40`, `0x41`)

**Alternative/legacy:** UART 24-channel servo controller (multi-servo grouped packet format)

### Servo Mapping (default layout)
- `LEG_SERVOS = [(0,1,2), (3,4,5), (6,7,8), (9,10,11), (12,13,14), (15,16,17)]`
- Logical angle → servo angle mapping:
  - `servo_deg = 90 + sign * logical_deg + offset`
  - `sign = -1` if channel inverted, else `+1`

### Safety Limits (bring-up)
- Recommended to start with tight limits (example: 30°–150°) until calibration is done
- Then widen only when you’re confident about linkage directions and neutral pose

---

## Repo Layout (Where to Look)

### `MCU_Micropython/` (recommended “robot deploy” folder)
Bring-up friendly MicroPython stack:
- `config.py` — geometry, mapping, calibration, I2C/PCA setup
- `servo_pca.py` — PCA9685 driver (servo_uart-compatible API)
- `ik.py` — IK + driver selection (`SERVO_DRIVER = "pca9685"` or `"uart"`)
- `frames.py` — body ↔ hip transforms
- `trajectory.py` — stance + swing paths
- `gait_tripod.py` — tripod scheduler + latching + phase-boundary replanning
- `main.py` — interactive loop (w/a/s/d/q/e/x) or demo mode

### `simulation/` (PC-side simulation)
A Python simulation that mirrors the real loop:
- `simulation/sim_main.py` — runs the gait loop and prints basic metrics
- `simulation/world.py`, `collision.py`, `robot_model.py` — simple evaluation of clearance/penetration

### Root-level scripts/docs (mixed / some legacy)
- HTML tools and planning docs
- Earlier UART-focused files also live here (kept for reference and quick tests)

---

## Quick Start

### A) MicroPython + PCA9685 (current intended path)
1. Copy `MCU_Micropython/*` onto the board.
2. Edit `MCU_Micropython/config.py`:
   - I2C pins + frequency
   - `PCA_ADDRS` (one or two boards)
   - `LEG_SERVOS`, `SERVO_INVERT`, `SERVO_OFFSET`, clamps
   - `hip_pos_B` (must match your chassis)
3. Run:
   - `import main`
   - Use keys: `w/s/a/d` (translate), `q/e` (yaw), `x` (stop)

> Note: there is an import mismatch to be aware of (see **Known Issues** below).

### B) UART 24-ch controller (alternative path)
Use `servo_uart.py` + `ik_fixed.py` for direct packet tests and IK motion checks.

---

## Known Issues / Cleanup Targets
- **Config naming mismatch:** `servo_pca.py` currently imports `config_pca` in some versions; the rest of the MicroPython stack prefers `config`.
  - Short-term workaround: duplicate/rename `config.py` → `config_pca.py`
  - Proper fix: make `servo_pca.py` import the same config module as everything else.
- **Duplicate configs in root vs MCU_Micropython:** root `config.py` contains older UART/IMU/BLE fields; MCU version is the PCA-focused bring-up config.
  - Plan: keep **one** “authoritative” config for the hardware path.

---

## Development Plan (Suggested Order)

### Phase 0 — Bring-up Safety
- Confirm power + wiring + channel mapping
- Move **one servo** slowly
- Center **all** servos safely (tight angle clamps)

### Phase 1 — IK Validation
- Single-leg IK sweep (small reachable box)
- All-legs same pose in one synchronized update (18 targets at once)

### Phase 2 — Neutral Stance + Calibration
- Tune `SERVO_INVERT` + `SERVO_OFFSET` until neutral stance is symmetric
- Lock per-servo min/max clamps that prevent self-collisions

### Phase 3 — Tripod Walking (Open Loop)
- Start with:
  - small step lengths
  - moderate clearance
  - zero yaw (`ω=0`)
- Add yaw after forward/backward is stable

### Phase 4 — Inputs + Smoothing
- Convert joystick/BLE packets into filtered `(v_x, v_y, ω)`
- Keep **latched** planning policy to avoid mid-swing target jumps

---

## Included Docs & Tools (in this repo / project bundle)
- Control logic “source of truth” (3-layer spec, frames, trajectories) :contentReference[oaicite:0]{index=0}  
- Build + validation order checklist :contentReference[oaicite:1]{index=1}  
- 2D gait + latched swing HTML visualizer :contentReference[oaicite:2]{index=2}  
- UART driver (legacy/alternate) :contentReference[oaicite:3]{index=3}  
- IK (standalone test version) :contentReference[oaicite:4]{index=4}  
- Quick MicroPython run notes (UART test) :contentReference[oaicite:5]{index=5}  
- YH-24 24-channel servo controller docs (schematic + manual) :contentReference[oaicite:6]{index=6} :contentReference[oaicite:7]{index=7}  

---

## Scope (What this repo is / isn’t)
- ✅ Flat-ground walking, open-loop, relative coordinates
- ✅ Clean separation of intent → trajectories → IK+IO
- ❌ Full SLAM / localization (not needed for tabletop MVP)
- ❌ Terrain adaptation (planned later, after stable walking)

---
