"""CLI runner for the MuJoCo hexapod simulation."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco

from simulation import config as sim_config
from simulation import gait_tripod
from simulation import ik

from . import bridge
from . import generate_mjcf
from . import logging_utils


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the MuJoCo hexapod simulation.")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation duration in seconds.")
    parser.add_argument("--dt", type=float, default=float(sim_config.DT), help="Simulation timestep in seconds.")
    parser.add_argument("--vx", type=float, default=0.0, help="Body forward velocity (mm/s).")
    parser.add_argument("--vy", type=float, default=0.0, help="Body lateral velocity (mm/s).")
    parser.add_argument("--wz", type=float, default=0.0, help="Body yaw rate (rad/s).")
    parser.add_argument("--real-time", dest="real_time", action="store_true", help="Run in real time.")
    parser.add_argument("--no-real-time", dest="real_time", action="store_false")
    parser.set_defaults(real_time=True)
    parser.add_argument("--headless", action="store_true", help="Run without the viewer.")
    parser.add_argument("--log", type=str, default=None, help="CSV log path.")
    return parser.parse_args()


def _build_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model_path = Path(__file__).resolve().parent / "models" / "hexapod.xml"
    if not model_path.exists():
        generate_mjcf.write_mjcf(model_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
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

    model, data = _build_model()
    dt = float(args.dt)

    gait = gait_tripod.TripodGait()
    gait.set_command(args.vx, args.vy, args.wz)

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
        _run_loop(model, data, gait, dt, args, state, logger=logger_ctx)
        if logger_ctx:
            logger_ctx.close()
        return

    import mujoco.viewer

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.key_callback = key_callback
        _run_loop(model, data, gait, dt, args, state, viewer=viewer, logger=logger_ctx)

    if logger_ctx:
        logger_ctx.close()


def _run_loop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    gait: gait_tripod.TripodGait,
    dt: float,
    args: argparse.Namespace,
    state: dict,
    viewer=None,
    logger: logging_utils.MjLogger | None = None,
) -> None:
    start = time.time()
    step_count = 0
    max_steps = int(args.duration / dt) if args.duration > 0 else None

    while state["running"]:
        if max_steps is not None and step_count >= max_steps:
            break

        if not state["paused"]:
            foot_targets = gait.update(dt=dt, send=False)
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
            target = step_count * dt
            sleep_time = target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    run()
