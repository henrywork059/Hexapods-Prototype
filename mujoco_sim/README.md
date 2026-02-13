# MuJoCo Simulation

This directory contains a lightweight MuJoCo model and runner wired into the existing gait/IK stack in `simulation/`.

## Install

Install MuJoCo Python bindings and dependencies (PC):

```bash
pip install -r requirements.txt
```

## Generate the MJCF

```bash
python -m mujoco_sim.generate_mjcf
```

This writes `mujoco_sim/models/hexapod.xml` using geometry from `simulation/config.py` (preferred) or `MCU_Micropython/config.py` (fallback).

## User Parameter Settings File

A user-editable settings file is available at `mujoco_sim/user_parameters.json`.

It lets you define key runtime values without editing Python code:

- Walking command defaults: `walk_command.vx`, `walk_command.vy`, `walk_command.wz`
- Single-leg debug values: `single_leg_debug.*`
- Robot leg/body geometry (mm): `robot_geometry_mm.*` (`coxa_length`, `femur_length`, `tibia_length`, `body_radius`, `stance_z0`)

Use a custom file with:

```bash
python -m mujoco_sim.run_mujoco --settings path/to/your_settings.json
```

CLI flags still override settings values when both are provided.

## Run with Viewer

```bash
python -m mujoco_sim.run_mujoco
```

This is the default runtime path: it starts in **tripod** mode and runs in real-time until you quit the viewer.
It exercises the full tripod gait + planner + IK + control mapping stack used by `simulation/sim_main.py`.

Useful options:

```bash
python -m mujoco_sim.run_mujoco --mode single-leg --swing-leg 0 --cycle-time 1.2 --step-length 30 --step-height 30
python -m mujoco_sim.run_mujoco --mode tripod --duration 10 --vx 20 --vy 0 --wz 0
```

Use `--mode single-leg` only as an optional diagnostics/debug mode when you want to cycle a single leg while the other five stay planted.

Controls in the viewer:

- **Space**: pause/unpause
- **R**: reset simulation
- **Q / Esc**: quit

## Headless Run + Logging

```bash
python -m mujoco_sim.run_mujoco --headless --duration 5 --vx 20 --log logs/run.csv
```

The CSV log includes time, torso pose, 18 joint targets, 18 joint positions, and foot site positions.

## Actuator Ordering

The MuJoCo control vector follows `mujoco_sim.bridge.JOINT_ORDER`:

```
leg0_coxa, leg0_femur, leg0_tibia,
leg1_coxa, leg1_femur, leg1_tibia,
...
leg5_coxa, leg5_femur, leg5_tibia
```

Actuator names are `act_<joint>` (e.g., `act_leg0_coxa`).
