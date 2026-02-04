# MCU_Micropython Test Plan

This plan targets MicroPython on-device execution with deterministic behavior. Use
`time.ticks_ms()` + `time.ticks_diff()` for timeouts and avoid long sleeps.

## Scope
Modules under `MCU_Micropython/`:
- `config.py`
- `frames.py`
- `trajectory.py`
- `gait_tripod.py`
- `ik.py`
- `servo_pca.py`
- `main.py`

## Module Categorization
**A) Pure logic/math (no hardware required)**
- `config.py` (constants + derived lists)
- `frames.py`
- `trajectory.py`

**B) Hardware-touching (needs MCU peripherals / servo driver)**
- `servo_pca.py` (I2C PCA9685)
- `ik.py` (drives servo backend via `servo_pca` or `servo_uart`)
- `gait_tripod.py` (calls IK → servo backend)
- `main.py` (USB-serial + gait loop)

## Test Matrix (concise)
| Module / function | Test file | Pass criteria |
| --- | --- | --- |
| `config.LOGICAL_TO_PCA`, `SERVO_INVERT`, `SERVO_OFFSET`, `SERVO_MIN_A`, `SERVO_MAX_A` | `tests/test_config.py` | Derived lists length == `NUM_LOGICAL_CHANNELS`; overrides applied correctly. |
| `frames.body_vec_to_hip`, `frames.hip_vec_to_body` | `tests/test_frames.py` | Round-trip within tolerance for known vectors/legs. |
| `frames.body_point_to_hip`, `frames.hip_point_to_body` | `tests/test_frames.py` | Round-trip within tolerance using `hip_pos_B`. |
| `frames.world_point_to_body`, `frames.body_point_to_world` | `tests/test_frames.py` | Round-trip within tolerance for nonzero pose. |
| `trajectory.stance` | `tests/test_trajectory.py` | Linear endpoints at phase 0/1 and monotonic interpolation. |
| `trajectory.swing_sine`, `swing_parabolic`, `swing_bezier` | `tests/test_trajectory.py` | Endpoints match; peak z >= endpoints + h*(expected), no NaNs. |
| `trajectory.swing` | `tests/test_trajectory.py` | Dispatches style correctly (same output as direct function). |
| `ik.logical_to_servo_angle` | `tests/test_ik_logic.py` | Inversion/offset/clamp rules applied correctly. |
| `ik.ik_xyz` | `tests/test_ik_logic.py` | Known reachable points produce finite angles within limits; `clamped_reach` set when outside reach. |
| `ik.move_leg_xyz` | `tests/test_ik_hw.py` | Sends 3 channels in one pose; angles within servo min/max. |
| `ik.move_all_legs_xyz` | `tests/test_ik_hw.py` | Sends 18 channels; rejects wrong-length target list. |
| `servo_pca.init` | `tests/test_servo_pca_hw.py` | I2C initialized; PCA9685 responds; frequency set without exception. |
| `servo_pca.set_servo` | `tests/test_servo_pca_hw.py` | Single channel write succeeds; no out-of-range ticks. |
| `servo_pca.set_pose` | `tests/test_servo_pca_hw.py` | Multiple channel write succeeds; angles interpolated if `t_ms`. |
| `servo_pca.center_all` | `tests/test_servo_pca_hw.py` | Moves configured channels to ~90° without error. |
| `gait_tripod.TripodGait.set_command` | `tests/test_gait_logic.py` | Command latching occurs only at phase boundary; deadband clamps small inputs. |
| `gait_tripod.TripodGait.update` (no send) | `tests/test_gait_logic.py` | Foot targets update deterministically for a fixed dt sequence. |
| `gait_tripod.TripodGait.reset_pose` (no send) | `tests/test_gait_logic.py` | Pose reset to neutral and replans phase. |
| `main.command_from_key` | `tests/test_main_logic.py` | Key inputs map to correct `set_command` calls. |
| `main.CommandSource.read_char` | `tests/test_main_hw.py` | Non-blocking read returns `None` when no input (USB-serial). |
| `main.run` | `tests/test_main_hw.py` | Demo path exits after `DEMO_SECONDS` with no exception. |

## Proposed test file layout
```
MCU_Micropython/tests/
  run_tests.py                # tiny MicroPython test runner
  fixtures.py                 # shared helpers (asserts, approx, timeout helper)
  test_config.py
  test_frames.py
  test_trajectory.py
  test_ik_logic.py
  test_gait_logic.py
  test_main_logic.py
  test_servo_pca_hw.py         # hardware-only
  test_ik_hw.py                # hardware-only
  test_main_hw.py              # hardware-only
```

## Recommended implementation order (pure → hardware)
1. `test_config.py`
2. `test_frames.py`
3. `test_trajectory.py`
4. `test_ik_logic.py`
5. `test_gait_logic.py`
6. `test_main_logic.py`
7. `test_servo_pca_hw.py`
8. `test_ik_hw.py`
9. `test_main_hw.py`

## Missing abstractions to enable safe testing (proposals only)
- **Servo backend interface**: Introduce a `servo_backend` shim (real PCA/UART vs. dry-run). This enables deterministic IK/gait tests without physical servo motion.
- **I2C/PCA9685 mock**: Provide a minimal `FakeI2C` for `servo_pca` unit tests to validate register writes without hardware.
- **Time provider**: Optional `time_provider` wrapper to control `ticks_ms()` for gait timing tests without real delays.
- **Config override hooks**: Allow tests to supply a temporary `config` module or overrides for geometry/limits without editing production files.
