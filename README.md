# Hexapods-Prototype — Tabletop Hexapod Walking Stack (MicroPython + Sim)

This repo is a prototype control stack for a **tabletop hexapod** (6 legs × 3DOF = **18 servos**), built around a testable pipeline:

**Intent (vx, vy, ω)** → **Gait planning + foot trajectories** → **IK + synchronized servo output**

The current MVP goal is **repeatable flat-ground walking** using **relative coordinates** (no external localization).

---

## 0. Contents

1. TL;DR — What to run
2. Current Status / Progress
3. Coordinate System & Conventions (LOCK THESE)
4. Architecture Overview (3 Layers)
5. “No World Frame” Walking (How it still moves)
6. Repository Layout (What matters)
7. Quick Start — Simulation (PC)
8. Quick Start — MicroPython (PCA9685 over I2C)
9. Quick Start — MuJoCo sim
10. Quick Start — Desktop GUI Viewer (sim_engine)
11. Built-in Test Harness (Recommended before moving real legs)
12. sim_engine Tests (Desktop)
13. Current Assumed Defaults (MVP)
14. Planning / Roadmap (Suggested order)
15. Known Issues / Sharp Edges
16. Internal Docs / References (in repo / project bundle)
17. Safety note

---

## 1. TL;DR — What to run

### Run on the robot (MicroPython + PCA9685 I2C)

Use: `MCU_Micropython/`

- **Entrypoint:** `MCU_Micropython/main.py`
- **Main config:** `MCU_Micropython/config.py`

### Validate on your PC (simulation)

Use: `simulation/`

- **Entrypoint:** `python -m simulation.sim_main`

### Browser visual tools (optional)

Open these HTML files directly in a browser:

- `2_d_hexapod_gait_sim_x_z_distance_driven_latched_swing_2_link_ik.html`
- (Other planner/UI prototypes in repo root)

---

## 2. Current Status / Progress

### Implemented (core MVP loop exists)

- **Tripod gait state machine** with two leg groups (A/B)
- **Phase-boundary command latching**
  - command updates apply **only at phase boundaries**
  - no mid-swing retarget (unless you add explicit reflex logic)
- **Swing + stance trajectory generation**
  - horizontal easing (linear/cubic/quintic)
  - swing lift styles: sine / parabolic / quadratic bezier
- **Hip-local IK (3DOF)**
  - hip origin at `(0,0,0)`
  - `z < 0` is **down under the hip**
  - knee-up branch so legs look like `/\`
  - coxa yaw limited to ±60° (default safety)
- **Servo backends**
  - PCA9685 I2C backend (intended primary)
  - UART backend exists (legacy/alternative; see Known Issues)
- **On-device test harness (MicroPython)**
  - `MCU_Micropython/tests/run_all.py` runs logic tests and optional hardware smoke tests

### Not integrated / deferred (not MVP)

- BLE input is not the default runtime path
- IMU leveling / terrain adaptation is out-of-scope for MVP
- Full world-frame locomotion is not required (relative gait is the target)

---

## 3. Coordinate System & Conventions (LOCK THESE)

These conventions are assumed throughout `MCU_Micropython/`.

### Frames

**Body frame (B)**

- origin: **body center**
- axes: `+x forward`, `+y left`, `+z up` (right-handed)
- yaw: positive = CCW (turn left) when viewed from above

**Hip-local frame (Hᵢ)** for each leg i

- origin: **hip joint**
- `+x`: radial outward from body center through that hip
- `+y`: leg-local left
- `+z`: up\
  → a foot below the hip has **negative z**

### IK input convention (IMPORTANT)

IK expects hip-local input: **(x, y, z) in mm**, with **z negative under the hip**.

---

## 4. Architecture Overview (3 Layers)

### Layer 1 — Intent → Phase Plan (Tripod gait)

Input (body command in body frame):

- `vx` (mm/s), `vy` (mm/s), `ω` (rad/s)

Output each tick:

- which legs are SWING vs STANCE
- per-leg phase plan generated at the **phase boundary**
- “delta per half-cycle” computed from translation + yaw

**Key behavior: Latched swing targets**

- the command is latched at a phase boundary
- each swing leg gets a fixed start + target for the entire swing

### Layer 2 — Trajectories (swing + stance)

**Stance**: interpolate `p0 → p1` (foot moves “backward” relative to body)

**Swing**:

- horizontal interpolation `p0 → p1` with easing
- vertical lift adds clearance `h`

Supported lift styles in `trajectory.py`:

- `sine` (smooth start/end)
- `parabolic` (simple mid-peak)
- `bezier` (tunable shape)

### Layer 3 — IK + Servo Output

Per leg:

- solve IK in hip-local frame → logical joint angles (deg)
- convert logical → servo degrees:
  - `servo_deg = 90 + sign * logical_deg + offset`
  - `sign = -1` if inverted else `+1`
  - clamp by per-servo min/max

Send:

- one combined pose update (18 channels) through selected backend

---

## 5. “No World Frame” Walking (How it still moves)

This prototype intentionally avoids requiring a global/world frame.

- During **STANCE**, we update each stance foot target so it stays “fixed” relative to the ground by moving it backward in the hip/body-relative sense.
- During **SWING**, we move the foot forward to the next planned landing point with lift.

Yaw is approximated by rotating foot points around the body origin (body frame), then mapping the delta into each hip frame.

This is sufficient for stable tabletop walking tests on flat ground (open loop).

---

## 6. Repository Layout (What matters)

### ✅ `MCU_Micropython/` (PRIMARY: deploy this to the board)

- `config.py` — hardware config + geometry + calibration + mapping (**edit this**)
- `main.py` — bring-up entrypoint (demo + optional interactive control)
- `gait_tripod.py` — tripod gait + latching + replanning at phase boundary
- `trajectory.py` — swing/stance path generators (math)
- `frames.py` — body↔hip transforms
- `ik.py` — hip-local IK + logical→servo conversion + backend output
- `servo_pca.py` — PCA9685 I2C driver (`set_pose(pairs, t_ms=...)`)
- `tests/` — MicroPython test harness + optional hardware smoke tests

### ✅ `simulation/` (PC: math validation)

- `simulation/sim_main.py` — runs gait + IK and prints basic metrics

### 🧪 Repo root (mixed / legacy / tooling)

Root-level code is not the recommended hardware deployment path.

- There is a root `config.py` that doesn’t match the PCA-first MicroPython config shape.
- Some root modules are experimental or older.

---

## 7. Quick Start — Simulation (PC)

From repo root:

```bash
python -m simulation.sim_main --duration 4 --speed-v 40 --speed-wz 0 --dt 0.02
```

Use the simulation to catch obvious parameter mistakes before moving hardware.

**Legacy matplotlib viewer:** the 3D matplotlib viewer in `simulation/viewer.py` is still available for the legacy simulation stack.

---

## 8. Quick Start — MicroPython (PCA9685 over I2C)

### Hardware assumptions

- MicroPython-capable MCU
- PCA9685 board(s) on I2C
- PCA frequency: 50 Hz
- typical servo pulses: 500–2500 µs
- 18 servos often uses **two PCA boards** (e.g. `0x40`, `0x41`)

### 1) Deploy the MCU stack to the board

Copy the contents of `MCU_Micropython/` to the device root so imports work.

### 2) Edit `MCU_Micropython/config.py` (MOST IMPORTANT)

Set:

- I2C pins (`I2C_ID`, `I2C_SDA`, `I2C_SCL`)
- PCA addresses (`PCA_ADDRS`)
- link lengths (`COXA_L`, `FEMUR_L`, `TIBIA_L`)
- hip positions in body frame (`HIP_POS_B`)
- servo mapping + calibration (`LEG_SERVOS`, invert/offset, min/max clamps)

### 3) Run

At the REPL:

```python
import main
main.run()
```

---

## 9. Quick Start — MuJoCo sim

Use: `mujoco_sim/`

```bash
python -m mujoco_sim.generate_mjcf
python -m mujoco_sim.run_mujoco --duration 10 --vx 20 --vy 0 --wz 0
```

To run headless with logs:

```bash
python -m mujoco_sim.run_mujoco --headless --duration 5 --vx 20 --log logs/mujoco_run.csv
```

---

## 10. Quick Start — Desktop GUI Viewer (sim_engine)

The desktop viewer renders the simplified 2D physics engine in `sim_engine/`.

From repo root:

```bash
python -m sim_engine
```

To launch the settings GUI (which saves a JSON settings file and then starts the viewer), run:

```bash
python -c "from sim_engine.viewer_desktop.app import ViewerApp; from sim_engine.viewer_desktop.settings_gui import launch_settings_gui; launch_settings_gui(lambda s: ViewerApp(s).root.mainloop())"
```

---

## 11. Built-in Test Harness (Recommended before moving real legs)

Run logic tests (no movement):

```python
import tests.run_all
tests.run_all.main()
```

If hardware smoke tests are enabled in the test plan, start with tight clamps and low speeds.

---

## 12. sim_engine Tests (Desktop)

From repo root:

```bash
python -m unittest discover sim_engine/tests
```

---

## 13. Current Assumed Defaults (MVP)

### Timing

- `DT = 0.02s` (50 Hz)
- `GAIT_PERIOD = 0.8s` (full cycle; half-cycle = 0.4s)

### Tripod groups (typical default)

The code assumes a consistent leg index order (documented in code). A common default:

- `TRIPOD_A = [0, 2, 4]`
- `TRIPOD_B = [1, 3, 5]`

### Neutral foot pose

If not overridden:

- `FOOT_NEUTRAL_H[i] = (80, 0, -80)` for each leg (hip-local)

### Step + lift clamps

- phase step clamp ~ 40 mm
- swing clearance starts ~ 25 mm and scales with step length (clamped)

### IK safety clamps

- coxa yaw clamp ±60°
- femur/tibia clamps set to prevent knee inversion and self-collision

---

## 14. Planning / Roadmap (Suggested order)

### Phase 0 — Bring-up safety

- verify I2C wiring + PCA address scan
- center servos with tight clamps
- confirm channel mapping

### Phase 1 — IK validation

- single-leg sweep in a small safe box
- all-legs neutral pose, then small moves

### Phase 2 — Calibration + neutral stance

- fix invert flags per channel
- tune per-servo offsets
- set safe min/max per channel

### Phase 3 — Tripod walking (open loop)

- slow forward only first
- increase step length gradually
- add yaw after stable forward motion

### Phase 4 — Inputs

- connect BLE / joystick
- keep phase-boundary latching to avoid mid-swing jumps

---

## 15. Known Issues / Sharp Edges

1. Some files may contain an extra first line with a single `\` (backslash), which breaks imports.

   - If you hit a syntax error at the first line, delete that `\` line.

2. `MCU_Micropython/servo_pca.py` may import `config_pca` in some versions.

   - Preferred fix: make it import `config` like the rest of the stack.
   - Quick workaround: create `config_pca.py` that re-exports `config`.

3. UART driver path may not match the newer `set_pose(pairs, t_ms=...)` signature.

   - For MVP, use the PCA9685 backend.

4. Root-level runtime code is not the recommended deployment path.

   - Use `MCU_Micropython/` for hardware; use `simulation/` for PC.

---

## 16. Internal Docs / References (in repo / project bundle)

- `hexapod_control_logic_canvas.md` — control logic and planning notes
- `hexapod_project_code_plan_build_validation_order.md` — build/validate order
- `Developing a Tabletop Hexapod Walking Robot_ Software & Theory Guide.pdf` — theory guide
- `docs/virtual_world_engine_desktop.md` — sim_engine architecture, API, solver math, and GUI details
- `docs/demos_desktop.md` — sim_engine desktop scene catalog
- `docs/CHANGELOG_sim_engine.md` — sim_engine changelog

---

## 17. Safety note

Start with tight servo limits, low speeds, and ideally lift the robot off the ground for first motion tests.
