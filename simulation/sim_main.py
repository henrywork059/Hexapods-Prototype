"""Simulation entrypoint mirroring the real main.py control loop."""

from __future__ import annotations

import argparse
import time

from . import config
from .gait_tripod import TripodGait
from . import ik
from .robot_model import RobotModel
from .world import World


SPEED_V = 40.0
SPEED_WZ = 0.6


def _sleep_s(dt_s: float):
    time.sleep(max(0.0, dt_s))


def run(duration: float, speed_v: float, speed_wz: float, dt: float):
    gait = TripodGait()
    gait.reset_pose(send=False)

    robot = RobotModel()
    world = World(body_pos=(0.0, 0.0, abs(config.STANCE_Z0) + 10.0))

    gait.set_command(speed_v, 0.0, speed_wz)

    t0 = time.time()
    last = t0
    step_idx = 0

    while True:
        now = time.time()
        dt_s = now - last
        last = now

        if dt_s <= 0.0 or dt_s > 0.2:
            dt_s = dt

        targets = gait.update(dt=dt_s, send=False)
        sols = ik.move_all_legs_xyz(targets, t_ms=int(max(10, dt_s * 1000)))
        robot.update_from_solutions(sols)

        metrics = world.evaluate(robot)
        if step_idx % 10 == 0:
            print(
                "step=%d min_ground=%.2fmm min_link=%.2fmm penetrations=%d"
                % (
                    step_idx,
                    metrics["min_ground_clearance"],
                    metrics["min_link_distance"],
                    metrics["ground_penetrations"],
                )
            )

        if duration > 0 and (now - t0) >= duration:
            break

        step_idx += 1
        _sleep_s(max(0.0, dt - dt_s))


def main():
    parser = argparse.ArgumentParser(description="Hexapod gait simulation (math-only)")
    parser.add_argument("--duration", type=float, default=4.0, help="seconds to run")
    parser.add_argument("--speed-v", type=float, default=SPEED_V, help="forward speed mm/s")
    parser.add_argument("--speed-wz", type=float, default=0.0, help="yaw rate rad/s")
    parser.add_argument("--dt", type=float, default=float(getattr(config, "DT", 0.02)))
    args = parser.parse_args()

    run(args.duration, args.speed_v, args.speed_wz, args.dt)


if __name__ == "__main__":
    main()
