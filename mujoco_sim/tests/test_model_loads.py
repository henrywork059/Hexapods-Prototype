import tempfile
from pathlib import Path

import mujoco

from mujoco_sim import generate_mjcf
from mujoco_sim import bridge


def test_model_loads_and_steps():
    with tempfile.TemporaryDirectory() as tmpdir:
        model_path = Path(tmpdir) / "hexapod.xml"
        generate_mjcf.write_mjcf(model_path)
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)

        joint_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(model.njnt)
        ]
        actuator_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(model.nu)
        ]

        assert model.njnt >= 18
        assert model.nu == 18
        for name in bridge.JOINT_ORDER:
            assert name in joint_names
        for name in bridge.ACTUATOR_ORDER:
            assert name in actuator_names

        data.ctrl[:] = 0.0
        for _ in range(100):
            mujoco.mj_step(model, data)
