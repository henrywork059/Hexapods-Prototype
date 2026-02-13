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


def test_load_settings_and_resolve_runtime_values(tmp_path):
    settings_path = tmp_path / "settings.json"
    settings_payload = {
        "walk_command": {"vx": 12, "vy": 3, "wz": 0.2},
        "single_leg_debug": {"swing_leg": 4, "cycle_time": 2.0, "step_length": 25, "step_height": 18},
        "robot_geometry_mm": {"coxa_length": 40.0},
    }
    settings_path.write_text(json.dumps(settings_payload))

    settings = run_mujoco._load_settings(str(settings_path))

    class _Args:
        vx = None
        vy = None
        wz = None
        swing_leg = None
        cycle_time = None
        step_length = None
        step_height = None

    runtime = run_mujoco._normalize_runtime_params(_Args(), settings)

    assert runtime["vx"] == 12.0
    assert runtime["vy"] == 3.0
    assert runtime["wz"] == 0.2
    assert runtime["swing_leg"] == 4
    assert runtime["cycle_time"] == 2.0
    assert runtime["step_length"] == 25.0
    assert runtime["step_height"] == 18.0
    assert runtime["geometry_overrides"] == {"coxa_length": 40.0}


def test_cli_values_override_settings_values():
    class _Args:
        vx = 99
        vy = None
        wz = None
        swing_leg = 1
        cycle_time = None
        step_length = 44
        step_height = None

    runtime = run_mujoco._normalize_runtime_params(
        _Args(),
        {
            "walk_command": {"vx": 10, "vy": 20, "wz": 0.5},
            "single_leg_debug": {"swing_leg": 4, "cycle_time": 2.0, "step_length": 25, "step_height": 18},
        },
    )

    assert runtime["vx"] == 99.0
    assert runtime["vy"] == 20.0
    assert runtime["wz"] == 0.5
    assert runtime["swing_leg"] == 1
    assert runtime["cycle_time"] == 2.0
    assert runtime["step_length"] == 44.0
    assert runtime["step_height"] == 18.0
