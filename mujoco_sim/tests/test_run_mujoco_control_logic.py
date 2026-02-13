import json

from mujoco_sim import run_mujoco


class _FakeGait:
    def __init__(self):
        self.calls: list[tuple[float, bool]] = []
        self.p_H = [(1.0, 2.0, -3.0)] * 6

    def update(self, dt: float, send: bool):
        self.calls.append((dt, send))
        return [(4.0, 5.0, -6.0)] * 6


def test_compute_foot_targets_tripod_uses_gait_update():
    gait = _FakeGait()
    neutral = list(gait.p_H)

    targets = run_mujoco._compute_foot_targets(
        mode="tripod",
        gait=gait,
        sim_dt=0.02,
        step_count=10,
        neutral_targets=neutral,
        swing_leg=0,
        cycle_time=1.2,
        step_length=30.0,
        step_height=30.0,
    )

    assert gait.calls == [(0.02, False)]
    assert targets == [(4.0, 5.0, -6.0)] * 6


def test_compute_foot_targets_single_leg_uses_local_cycle_path():
    gait = _FakeGait()
    neutral = list(gait.p_H)

    targets = run_mujoco._compute_foot_targets(
        mode="single-leg",
        gait=gait,
        sim_dt=0.02,
        step_count=10,
        neutral_targets=neutral,
        swing_leg=2,
        cycle_time=1.2,
        step_length=30.0,
        step_height=30.0,
    )

    assert gait.calls == []
    assert targets != neutral
    for idx, target in enumerate(targets):
        if idx == 2:
            continue
        assert target == neutral[idx]


def test_parse_args_defaults_to_tripod_mode(monkeypatch):
    monkeypatch.setattr("sys.argv", ["run_mujoco"])

    args = run_mujoco._parse_args()

    assert args.mode == "tripod"
