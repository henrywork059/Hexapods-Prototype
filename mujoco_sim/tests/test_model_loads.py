import tempfile
from pathlib import Path

import mujoco
import numpy as np

from mujoco_sim import bridge
from mujoco_sim import generate_mjcf


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

        assert len(bridge.JOINT_ORDER) == 18
        assert len(bridge.ACTUATOR_ORDER) == 18
        assert model.nu == 18
        assert data.ctrl.shape[0] == 18
        for name in bridge.JOINT_ORDER:
            assert name in joint_names
        for name in bridge.ACTUATOR_ORDER:
            assert name in actuator_names

        data.ctrl[:] = 0.0
        for _ in range(100):
            mujoco.mj_step(model, data)
            assert np.all(np.isfinite(data.qpos))
            assert np.all(np.isfinite(data.qvel))


def test_generated_model_uses_requested_timestep():
    with tempfile.TemporaryDirectory() as tmpdir:
        model_path = Path(tmpdir) / "hexapod.xml"
        requested_dt = 0.003
        generate_mjcf.write_mjcf(model_path, timestep=requested_dt)

        model = mujoco.MjModel.from_xml_path(str(model_path))

        assert model.opt.timestep == requested_dt
