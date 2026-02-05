"""3D visual viewer for the Python simulation.

Run from repo root:
  python -m simulation.viewer --duration 0 --speed-v 40 --speed-wz 0.0

Controls (Matplotlib 3D defaults):
  - Left-drag: rotate
  - Scroll: zoom
  - Right/Middle drag: pan (depends on backend)
"""

from __future__ import annotations

import argparse
import math
import time
from typing import List, Sequence, Tuple

Vector = Tuple[float, float, float]


def _ensure_gui_backend(preferred: str | None = None) -> None:
    """
    If matplotlib is using a non-GUI backend (like Agg), force a GUI backend
    so a real window opens on Windows/macOS/Linux.
    Must be called BEFORE importing matplotlib.pyplot.
    """
    import matplotlib

    if preferred:
        try:
            matplotlib.use(preferred, force=True)
            return
        except Exception:
            pass

    backend = matplotlib.get_backend().lower()
    if "agg" not in backend:
        return

    # Try common GUI backends in a safe order.
    for cand in ("TkAgg", "QtAgg", "Qt5Agg"):
        try:
            matplotlib.use(cand, force=True)
            return
        except Exception:
            continue
    # If none worked, matplotlib will stay on Agg (no window).


def _polyline_from_segments(segments: Sequence[Tuple[Vector, Vector]]) -> List[Vector]:
    if not segments:
        return []
    pts: List[Vector] = [segments[0][0]]
    for _p, q in segments:
        pts.append(q)
    return pts


def _make_body_outline_B(radius: float, z: float = 0.0, n: int = 6) -> List[Vector]:
    pts: List[Vector] = []
    n = max(3, int(n))
    for k in range(n + 1):
        a = 2.0 * math.pi * (k / n)
        pts.append((radius * math.cos(a), radius * math.sin(a), z))
    return pts


def run(duration: float, speed_v: float, speed_wz: float, dt: float, fps: float, steps_per_frame: int, backend: str):
    _ensure_gui_backend(backend if backend else None)

    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    from simulation import config, frames, ik
    from simulation.gait_tripod import TripodGait
    from simulation.robot_model import RobotModel
    from simulation.world import World

    # ---- Sim state ----
    gait = TripodGait()
    gait.reset_pose(send=False)
    gait.set_command(speed_v, 0.0, speed_wz)

    robot = RobotModel()
    world = World(body_pos=(0.0, 0.0, abs(config.STANCE_Z0) + 10.0))

    # ---- Figure ----
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    view_r = float(
        getattr(
            config,
            "VIEW_RADIUS",
            (config.HIP_RADIUS + config.COXA_L + config.FEMUR_L + config.TIBIA_L + 40.0),
        )
    )

    # Ground rectangle outline
    r = view_r
    gx = [-r, r, r, -r, -r]
    gy = [-r, -r, r, r, -r]
    gz = [world.ground_z] * 5
    (ground_line,) = ax.plot(gx, gy, gz, linewidth=1)

    # Body outline
    body_outline_B = _make_body_outline_B(float(getattr(config, "BODY_RADIUS", config.HIP_RADIUS)), z=0.0, n=6)
    body_outline_W = [frames.body_point_to_world(p, world.body_pos, world.body_rpy) for p in body_outline_B]
    bx, by, bz = zip(*body_outline_W)
    (body_line,) = ax.plot(bx, by, bz, linewidth=2)

    # Hip markers
    hips_B: List[Vector] = list(getattr(config, "hip_pos_B", []))
    hips_W = [frames.body_point_to_world(p, world.body_pos, world.body_rpy) for p in hips_B]
    hx = [p[0] for p in hips_W]
    hy = [p[1] for p in hips_W]
    hz = [p[2] for p in hips_W]
    hip_scatter = ax.scatter(hx, hy, hz, s=18)

    # Leg polylines (6)
    leg_lines = []
    for _ in range(6):
        (ln,) = ax.plot([0, 0], [0, 0], [0, 0], linewidth=2)
        leg_lines.append(ln)

    # Foot markers
    foot_scatter = ax.scatter([], [], [], s=28)

    # Axes setup
    ax.set_xlim(-r, r)
    ax.set_ylim(-r, r)
    ax.set_zlim(world.ground_z - 30.0, world.body_pos[2] + 60.0)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    try:
        ax.set_box_aspect((1, 1, 0.7))
    except Exception:
        pass
    ax.view_init(elev=20, azim=-60)

    # ---- Simulation step ----
    def _sim_step(dt_s: float):
        targets = gait.update(dt=dt_s, send=False)
        sols = ik.move_all_legs_xyz(targets, t_ms=int(max(10, dt_s * 1000)))
        robot.update_from_solutions(sols)
        return world.evaluate(robot)

    start_t = time.time()

    def _update(_frame_idx: int):
        elapsed = time.time() - start_t
        if duration > 0 and elapsed >= duration:
            plt.close(fig)
            return []

        metrics = None
        for _ in range(max(1, int(steps_per_frame))):
            metrics = _sim_step(float(dt))

        segs_by_leg = robot.segments_by_leg()
        for i, segs in enumerate(segs_by_leg):
            pts_B = _polyline_from_segments(segs)
            pts_W = [frames.body_point_to_world(p, world.body_pos, world.body_rpy) for p in pts_B]
            xs = [p[0] for p in pts_W]
            ys = [p[1] for p in pts_W]
            zs = [p[2] for p in pts_W]
            leg_lines[i].set_data(xs, ys)
            leg_lines[i].set_3d_properties(zs)

        feet_B = robot.foot_positions_body()
        feet_W = [frames.body_point_to_world(p, world.body_pos, world.body_rpy) for p in feet_B]
        fx = [p[0] for p in feet_W]
        fy = [p[1] for p in feet_W]
        fz = [p[2] for p in feet_W]
        foot_scatter._offsets3d = (fx, fy, fz)

        if metrics is None:
            metrics = world.evaluate(robot)

        ax.set_title(
            f"backend={matplotlib.get_backend()}  "
            f"min_ground={metrics['min_ground_clearance']:.1f}mm  "
            f"min_link={metrics['min_link_distance']:.1f}mm  "
            f"penetrations={metrics['ground_penetrations']:.0f}"
        )

        return [ground_line, body_line, hip_scatter, foot_scatter, *leg_lines]

    interval_ms = int(1000.0 / max(1.0, float(fps)))

    # IMPORTANT: keep a reference to the animation object
    anim = FuncAnimation(
        fig,
        _update,
        interval=interval_ms,
        blit=False,
        cache_frame_data=False,  # avoids unbounded caching warning
    )

    plt.show()  # should block and keep window alive
    _ = anim  # keep reference explicit


def main():
    parser = argparse.ArgumentParser(description="3D viewer for the Python simulation")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to run (0 = run until closed)")
    parser.add_argument("--speed-v", type=float, default=40.0, help="forward speed mm/s")
    parser.add_argument("--speed-wz", type=float, default=0.0, help="yaw rate rad/s")
    parser.add_argument("--dt", type=float, default=0.02, help="simulation timestep seconds")
    parser.add_argument("--fps", type=float, default=30.0, help="viewer refresh rate")
    parser.add_argument("--steps-per-frame", type=int, default=1, help="sim steps per rendered frame")
    parser.add_argument(
        "--backend",
        type=str,
        default="",
        help="force matplotlib backend (e.g. TkAgg, QtAgg). Leave empty for auto.",
    )
    args = parser.parse_args()

    run(args.duration, args.speed_v, args.speed_wz, args.dt, args.fps, args.steps_per_frame, args.backend)


if __name__ == "__main__":
    main()
