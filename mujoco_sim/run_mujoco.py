"""CLI runner for the MuJoCo hexapod simulation."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco

from simulation import config as sim_config
from simulation import gait_tripod
from simulation import ik

from . import bridge
from . import generate_mjcf
from . import logging_utils


DEFAULT_SETTINGS_PATH = Path(__file__).resolve().parent / "user_parameters.json"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the MuJoCo hexapod simulation.")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Simulation duration in seconds. Use <= 0 to run until you close the viewer.",
    )
    parser.add_argument("--dt", type=float, default=float(sim_config.DT), help="Simulation timestep in seconds.")
    parser.add_argument("--settings", type=str, default=str(DEFAULT_SETTINGS_PATH), help="Path to user parameter settings JSON.")
    parser.add_argument("--vx", type=float, default=None, help="Body forward velocity (mm/s).")
    parser.add_argument("--vy", type=float, default=None, help="Body lateral velocity (mm/s).")
    parser.add_argument("--wz", type=float, default=None, help="Body yaw rate (rad/s).")
    parser.add_argument("--real-time", dest="real_time", action="store_true", help="Run in real time.")
    parser.add_argument("--no-real-time", dest="real_time", action="store_false")
    parser.set_defaults(real_time=True)
    parser.add_argument("--headless", action="store_true", help="Run without the viewer.")
    parser.add_argument("--log", type=str, default=None, help="CSV log path.")
    parser.add_argument(
        "--mode",
        choices=("single-leg", "tripod"),
        default="tripod",
        help="Control mode: full tripod gait (default) or simplified single-leg debug gait.",
    )
    parser.add_argument("--swing-leg", type=int, default=None, help="Leg index to cycle in single-leg mode (0-5).")
    parser.add_argument("--cycle-time", type=float, default=None, help="Single-leg gait cycle period in seconds.")
    parser.add_argument("--step-length", type=float, default=None, help="Single-leg forward/back sweep in mm.")
    parser.add_argument("--step-height", type=float, default=None, help="Single-leg lift height in mm.")
    return parser.parse_args()


def _load_settings(path: str) -> dict:
    settings_path = Path(path)
    if not settings_path.exists():
        return {}
    try:
        return json.loads(settings_path.read_text())
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid settings JSON: {settings_path}") from exc


def _resolve_setting(cli_value, settings: dict, dotted_key: str, fallback):
    if cli_value is not None:
        return cli_value

    node = settings
    for key in dotted_key.split('.'):
        if not isinstance(node, dict) or key not in node:
            return fallback
        node = node[key]

    return node


def _normalize_runtime_params(args: argparse.Namespace, settings: dict) -> dict[str, float | int | dict[str, float]]:
    params = {
        "vx": float(_resolve_setting(args.vx, settings, "walk_command.vx", 0.0)),
        "vy": float(_resolve_setting(args.vy, settings, "walk_command.vy", 0.0)),
        "wz": float(_resolve_setting(args.wz, settings, "walk_command.wz", 0.0)),
        "swing_leg": int(_resolve_setting(args.swing_leg, settings, "single_leg_debug.swing_leg", 0)),
        "cycle_time": float(_resolve_setting(args.cycle_time, settings, "single_leg_debug.cycle_time", 1.2)),
        "step_length": float(_resolve_setting(args.step_length, settings, "single_leg_debug.step_length", 30.0)),
        "step_height": float(_resolve_setting(args.step_height, settings, "single_leg_debug.step_height", 30.0)),
        "geometry_overrides": _resolve_setting(None, settings, "robot_geometry_mm", {}),
    }
    if not isinstance(params["geometry_overrides"], dict):
        params["geometry_overrides"] = {}
    return params


def _build_model(dt: float, geometry_overrides: dict[str, float] | None = None) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model_path = Path(__file__).resolve().parent / "models" / "hexapod.xml"
    if not model_path.exists():
        generate_mjcf.write_mjcf(model_path, timestep=dt, geometry_overrides=geometry_overrides)
    else:
        existing_model = mujoco.MjModel.from_xml_path(str(model_path))
        if abs(existing_model.opt.timestep - dt) > 1e-12:
            generate_mjcf.write_mjcf(model_path, timestep=dt, geometry_overrides=geometry_overrides)
    if geometry_overrides:
        generate_mjcf.write_mjcf(model_path, timestep=dt, geometry_overrides=geometry_overrides)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = dt
    data = mujoco.MjData(model)
    return model, data


def _set_neutral_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    gait: gait_tripod.TripodGait,
) -> None:
    gait.reset_pose(send=False)
    foot_targets = list(gait.p_H)
    ik_solutions = [ik.ik_xyz(*target) for target in foot_targets]
    ctrl = bridge.map_ik_to_ctrl(ik_solutions, degrees=True)

    for joint_name, target in zip(bridge.JOINT_ORDER, ctrl):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            continue
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        qpos_addr = model.jnt_qposadr[joint_id]
        data.qpos[qpos_addr] = target

    for actuator_name, target in zip(bridge.ACTUATOR_ORDER, ctrl):
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if actuator_id < 0:
            continue
        data.ctrl[actuator_id] = target

    mujoco.mj_forward(model, data)


def run() -> None:
    args = _parse_args()

    settings = _load_settings(args.settings)
    runtime = _normalize_runtime_params(args, settings)

    dt = float(args.dt)
    model, data = _build_model(dt, geometry_overrides=runtime["geometry_overrides"])

    gait = gait_tripod.TripodGait()
    gait.set_command(runtime["vx"], runtime["vy"], runtime["wz"])

    state = {"paused": False, "running": True}
    _set_neutral_pose(model, data, gait)

    def key_callback(keycode):
        nonlocal state
        if keycode in (ord(" "),):
            state["paused"] = not state["paused"]
        elif keycode in (ord("r"), ord("R")):
            mujoco.mj_resetData(model, data)
            _set_neutral_pose(model, data, gait)
        elif keycode in (ord("q"), ord("Q"), 27):
            state["running"] = False

    logger_ctx = (
        logging_utils.MjLogger(model, data, args.log, log_sites=True)
        if args.log
        else None
    )

    if args.headless:
        _run_loop(model, data, gait, args, state, logger=logger_ctx, runtime=runtime)
        if logger_ctx:
            logger_ctx.close()
        return

    import mujoco.viewer

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.key_callback = key_callback
        _run_loop(model, data, gait, args, state, viewer=viewer, logger=logger_ctx, runtime=runtime)

    if logger_ctx:
        logger_ctx.close()


def _run_loop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    gait: gait_tripod.TripodGait,
    args: argparse.Namespace,
    state: dict,
    viewer=None,
    logger: logging_utils.MjLogger | None = None,
    runtime: dict | None = None,
) -> None:
    runtime = runtime or {}
    sim_dt = float(model.opt.timestep)
    neutral_targets = list(gait.p_H)
    swing_leg = max(0, min(5, int(runtime["swing_leg"])))
    start = time.time()
    step_count = 0
    max_steps = int(args.duration / sim_dt) if args.duration > 0 else None

    while state["running"]:
        if max_steps is not None and step_count >= max_steps:
            break

        if not state["paused"]:
            foot_targets = _compute_foot_targets(
                mode=args.mode,
                gait=gait,
                sim_dt=sim_dt,
                step_count=step_count,
                neutral_targets=neutral_targets,
                swing_leg=swing_leg,
                cycle_time=float(runtime["cycle_time"]),
                step_length=float(runtime["step_length"]),
                step_height=float(runtime["step_height"]),
                cycle_time=float(args.cycle_time),
                step_length=float(args.step_length),
                step_height=float(args.step_height),
            )
            ik_solutions = [ik.ik_xyz(*target) for target in foot_targets]
            ctrl = bridge.map_ik_to_ctrl(ik_solutions, degrees=True)
            data.ctrl[:] = ctrl
            mujoco.mj_step(model, data)
            if logger:
                logger.log()
            step_count += 1

        if viewer is not None:
            viewer.sync()

        if args.real_time:
            elapsed = time.time() - start
            target = step_count * sim_dt
            sleep_time = target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


def _compute_foot_targets(
    mode: str,
    gait: gait_tripod.TripodGait,
    sim_dt: float,
    step_count: int,
    neutral_targets: list[tuple[float, float, float]],
    swing_leg: int,
    cycle_time: float,
    step_length: float,
    step_height: float,
) -> list[tuple[float, float, float]]:
    if mode == "single-leg":
        return _single_leg_targets(
            t=step_count * sim_dt,
            neutral=neutral_targets,
            leg_index=swing_leg,
            cycle_time=max(sim_dt, cycle_time),
            step_length=step_length,
            step_height=step_height,
        )

    return gait.update(dt=sim_dt, send=False)


def _single_leg_targets(
    t: float,
    neutral: list[tuple[float, float, float]],
    leg_index: int,
    cycle_time: float,
    step_length: float,
    step_height: float,
) -> list[tuple[float, float, float]]:
    """Generate a simplified gait where one leg cycles while all others stay planted."""
    targets = [tuple(p) for p in neutral]

    phase = (t / cycle_time) % 1.0
    x0, y0, z0 = targets[leg_index]

    if phase < 0.5:
        u = phase / 0.5
        x = x0 + (u - 0.5) * step_length
        z = z0 + math.sin(math.pi * u) * step_height
    else:
        u = (phase - 0.5) / 0.5
        x = x0 + (0.5 - u) * step_length
        z = z0

    targets[leg_index] = (x, y0, z)
    return targets


if __name__ == "__main__":
    run()
