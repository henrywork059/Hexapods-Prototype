diff --git a/README.md b/README.md
index 6c5d947f9fbb1669ee0468fedd7f3e24c79bbcf2..0d5cde0526859a9aa097f50ed631b72eac6b5af1 100644
--- a/README.md
+++ b/README.md
@@ -1 +1,54 @@
-# Hexapods-Prototype
\ No newline at end of file
+# Hexapods-Prototype
+
+Prototype control stack for a tabletop hexapod, focused on a clean 3-layer walking architecture:
+1) Intent → gait targets, 2) foot trajectories, 3) IK + synchronized servo output. The design
+keeps everything in the body frame (no world frame) by maintaining stance anchors during
+stance phases and generating smooth swing trajectories in body coordinates. 【F:hexapod_control_logic_canvas.md†L1-L299】
+
+## Architecture Overview
+
+### Coordinate Conventions
+- **Body frame (B):** origin at body center; +x forward, +y left, +z up, right-handed. Yaw
+  is CCW for positive values. 【F:hexapod_control_logic_canvas.md†L8-L40】
+- **Hip-local frame (Hᵢ):** origin at each hip; +x radial outward from body center, +y tangential
+  CCW, +z up (foot below hip is negative z). 【F:hexapod_control_logic_canvas.md†L42-L63】
+
+### Layer 1 — Intent → Gait Targets
+Inputs are body-frame velocity `(v_x, v_y)` and yaw rate `ω`. Output per-leg phase (STANCE/SWING)
+and swing endpoints `p0_B` and `p1_B` in body coordinates. Stance anchors `p_anchor_B` are updated
+each tick to keep stance feet fixed relative to the ground. 【F:hexapod_control_logic_canvas.md†L83-L124】
+
+### Layer 2 — Foot Trajectory Generator
+Stance legs reuse `p_anchor_B`. Swing legs use a cubic smoothstep for XY interpolation and a
+4th-order bump for vertical clearance, producing a smooth swing arc. 【F:hexapod_control_logic_canvas.md†L126-L214】
+
+### Layer 3 — IK + Synchronized Servo Output
+Body-frame targets are translated to the hip origin and rotated by the hip yaw to produce hip-local
+targets, then solved via IK and emitted in one grouped servo packet. 【F:hexapod_control_logic_canvas.md†L216-L256】
+
+## Repository Layout
+Key files in this prototype:
+- `main.py`: glue loop (tick update and IK send).
+- `config.py`: geometry, calibration, and servo mapping.
+- `gait_tripod.py`: tripod gait schedule and swing planning.
+- `ik_fixed.py`: hip-local IK and multi-leg movement.
+- `servo_uart.py`: UART packet output for grouped moves.
+- `hexapod_control_logic_canvas.md`: canonical control logic doc (source of architecture). 【F:hexapod_control_logic_canvas.md†L1-L299】
+
+## How the Control Loop Works (Tick)
+At ~50–100 Hz:
+1) Read command `(v_x, v_y, ω)` and compute per-tick deltas.
+2) Update gait phases and swing endpoints.
+3) Update stance anchors to keep stance feet ground-fixed.
+4) Compute desired foot positions (stance or swing).
+5) Transform to hip-local, solve IK, send a single grouped servo packet. 【F:hexapod_control_logic_canvas.md†L258-L289】
+
+## Minimum Setup Checklist
+Before moving:
+1) Populate `config.hip_pos_B` and `config.hip_yaw0` with real geometry.
+2) Define neutral stance anchors in `main.py` (spread XY, set `z0`).
+3) Set tripod group indices in `gait_tripod.py`.
+4) Tune `z_body`, `h_clear`, `step_len`, `T`, and `beta`. 【F:hexapod_control_logic_canvas.md†L445-L452】
+
+## References
+- `hexapod_control_logic_canvas.md` for full design notes and MVP file templates. 【F:hexapod_control_logic_canvas.md†L1-L452】
