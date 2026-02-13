import tempfile
from pathlib import Path

import mujoco
import numpy as np

from mujoco_sim import bridge
from mujoco_sim import generate_mjcf
from simulation import ik
from simulation.gait_tripod import TripodGait


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


def test_neutral_tripod_targets_are_kinematically_consistent():
    with tempfile.TemporaryDirectory() as tmpdir:
        model_path = Path(tmpdir) / "hexapod.xml"
        generate_mjcf.write_mjcf(model_path)

        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)

        gait = TripodGait()
        gait.reset_pose(send=False)
        ik_solutions = [ik.ik_xyz(*target_hip) for target_hip in gait.p_H]

        ctrl = bridge.map_ik_to_ctrl(ik_solutions)
        data.ctrl[:] = ctrl

        expected_femur_signs = [np.sign(sol["femur"]) for sol in ik_solutions]
        expected_tibia_signs = [np.sign(sol["tibia"]) for sol in ik_solutions]
        commanded_femur_signs = [np.sign(ctrl[3 * leg + 1]) for leg in range(6)]
        commanded_tibia_signs = [np.sign(ctrl[3 * leg + 2]) for leg in range(6)]

        assert all(sign <= 0 for sign in expected_femur_signs)
        assert all(sign < 0 for sign in expected_tibia_signs)
        assert commanded_femur_signs == expected_femur_signs
        assert commanded_tibia_signs == expected_tibia_signs

        for _ in range(20):
            mujoco.mj_step(model, data)

        torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        torso_z = float(data.xpos[torso_id][2])
        rel_foot_z = []
        for leg in range(6):
            site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"foot_leg{leg}")
            rel_foot_z.append(float(data.site_xpos[site_id][2] - torso_z))

        assert all(z < -1e-3 for z in rel_foot_z)


def test_generated_model_uses_geometry_overrides():
    with tempfile.TemporaryDirectory() as tmpdir:
        model_path = Path(tmpdir) / "hexapod.xml"
        generate_mjcf.write_mjcf(
            model_path,
            geometry_overrides={"coxa_length": 50.0, "femur_length": 70.0, "tibia_length": 90.0},
        )

        model = mujoco.MjModel.from_xml_path(str(model_path))

        coxa_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "leg0_coxa")
        femur_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "leg0_femur_body")
        tibia_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "leg0_tibia_body")

        coxa_qpos_addr = model.jnt_qposadr[coxa_joint]
        assert coxa_qpos_addr >= 0
        assert np.isclose(model.body_pos[femur_body][0], 0.05)
        assert np.isclose(model.body_pos[tibia_body][0], 0.07)
