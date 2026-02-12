"""Generate MJCF for the hexapod MuJoCo model."""

from __future__ import annotations

import importlib
import math
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET


def _load_config():
    try:
        return importlib.import_module("simulation.config")
    except Exception:
        try:
            return importlib.import_module("MCU_Micropython.config")
        except Exception as exc:
            raise RuntimeError("Unable to import simulation.config or MCU_Micropython.config") from exc


def _get(cfg, name: str, default):
    return getattr(cfg, name, default)


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _vec_to_str(vec: Iterable[float]) -> str:
    return " ".join(f"{v:.6f}" for v in vec)


def build_mjcf_xml(timestep: float = 0.01) -> ET.Element:
    cfg = _load_config()

    coxa_l = float(_get(cfg, "COXA_L", 34.5)) * 0.001
    femur_l = float(_get(cfg, "FEMUR_L", 64.0)) * 0.001
    tibia_l = float(_get(cfg, "TIBIA_L", 86.71)) * 0.001

    body_radius = float(_get(cfg, "BODY_RADIUS", 80.0)) * 0.001
    stance_z0 = float(_get(cfg, "STANCE_Z0", -80.0)) * 0.001

    hip_pos = _get(cfg, "hip_pos_B", None)
    mount_angles = _get(cfg, "MOUNT_ANGLES", None)

    if hip_pos is None:
        hip_radius = float(_get(cfg, "HIP_RADIUS", body_radius * 1000.0)) * 0.001
        if mount_angles is None:
            mount_angles = [
                math.pi / 6,
                math.pi / 2,
                5 * math.pi / 6,
                7 * math.pi / 6,
                3 * math.pi / 2,
                11 * math.pi / 6,
            ]
        hip_pos = [(hip_radius * math.cos(theta), hip_radius * math.sin(theta), 0.0) for theta in mount_angles]
    else:
        hip_pos = [(float(x) * 0.001, float(y) * 0.001, float(z) * 0.001) for x, y, z in hip_pos]

    if mount_angles is None:
        mount_angles = [math.atan2(y, x) for x, y, _ in hip_pos]

    torso_z = max(0.08, -stance_z0 + 0.02)

    root = ET.Element("mujoco", model="hexapod")
    compiler = ET.SubElement(root, "compiler", angle="radian", coordinate="local")
    ET.SubElement(
        root,
        "option",
        timestep="0.003",
        gravity="0 0 -9.81",
        integrator="implicitfast",
        iterations="80",
        ls_iterations="20",
    )
    ET.SubElement(root, "option", timestep=f"{float(timestep):.9g}", gravity="0 0 -9.81")

    default = ET.SubElement(root, "default")
    ET.SubElement(
        default,
        "geom",
        type="capsule",
        size="0.01",
        density="500",
        rgba="0.6 0.6 0.6 1",
        friction="1.0 0.02 0.002",
        solref="0.004 1",
        solimp="0.90 0.95 0.001",
    )
    ET.SubElement(default, "joint", damping="2.5", armature="0.01", limited="true")
    ET.SubElement(default, "position", kp="20", kv="9")

    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        name="floor",
        type="plane",
        size="5 5 0.1",
        rgba="0.2 0.3 0.4 1",
    )

    torso = ET.SubElement(worldbody, "body", name="torso", pos=_vec_to_str((0.0, 0.0, torso_z)))
    ET.SubElement(torso, "freejoint")
    ET.SubElement(
        torso,
        "geom",
        name="torso_geom",
        type="cylinder",
        size=_vec_to_str((body_radius, 0.02)),
        rgba="0.4 0.4 0.4 1",
    )

    for i, ((x, y, z), yaw) in enumerate(zip(hip_pos, mount_angles)):
        leg_body = ET.SubElement(
            torso,
            "body",
            name=f"leg{i}_hip",
            pos=_vec_to_str((x, y, z)),
            euler=_vec_to_str((0.0, 0.0, yaw)),
        )
        ET.SubElement(
            leg_body,
            "joint",
            name=f"leg{i}_coxa",
            type="hinge",
            axis="0 0 1",
            range=_vec_to_str((_deg_to_rad(-60.0), _deg_to_rad(60.0))),
        )
        ET.SubElement(
            leg_body,
            "geom",
            name=f"leg{i}_coxa_geom",
            fromto=_vec_to_str((0.0, 0.0, 0.0, coxa_l, 0.0, 0.0)),
            rgba="0.8 0.4 0.4 1",
        )

        femur_body = ET.SubElement(
            leg_body,
            "body",
            name=f"leg{i}_femur_body",
            pos=_vec_to_str((coxa_l, 0.0, 0.0)),
        )
        ET.SubElement(
            femur_body,
            "joint",
            name=f"leg{i}_femur",
            type="hinge",
            # Match simulation/ik.py convention: z negative is down, so positive
            # femur/tibia IK angles should move the links downward in MuJoCo too.
            axis="0 -1 0",
            range=_vec_to_str((_deg_to_rad(-90.0), _deg_to_rad(90.0))),
        )
        ET.SubElement(
            femur_body,
            "geom",
            name=f"leg{i}_femur_geom",
            fromto=_vec_to_str((0.0, 0.0, 0.0, femur_l, 0.0, 0.0)),
            rgba="0.4 0.8 0.4 1",
        )

        tibia_body = ET.SubElement(
            femur_body,
            "body",
            name=f"leg{i}_tibia_body",
            pos=_vec_to_str((femur_l, 0.0, 0.0)),
        )
        ET.SubElement(
            tibia_body,
            "joint",
            name=f"leg{i}_tibia",
            type="hinge",
            # Keep range limits aligned with simulation/ik.py limits; only hinge
            # axis is flipped to align joint sign semantics across sim stacks.
            axis="0 -1 0",
            range=_vec_to_str((_deg_to_rad(-140.0), _deg_to_rad(0.0))),
        )
        ET.SubElement(
            tibia_body,
            "geom",
            name=f"leg{i}_tibia_geom",
            fromto=_vec_to_str((0.0, 0.0, 0.0, tibia_l, 0.0, 0.0)),
            rgba="0.4 0.4 0.8 1",
        )
        ET.SubElement(
            tibia_body,
            "site",
            name=f"foot_leg{i}",
            pos=_vec_to_str((tibia_l, 0.0, 0.0)),
            size="0.01",
            rgba="0.2 0.9 0.9 1",
        )

    actuator = ET.SubElement(root, "actuator")
    for i in range(6):
        for joint_name, lo, hi in (
            (f"leg{i}_coxa", -60.0, 60.0),
            (f"leg{i}_femur", -90.0, 90.0),
            (f"leg{i}_tibia", -140.0, 0.0),
        ):
            ET.SubElement(
                actuator,
                "position",
                name=f"act_{joint_name}",
                joint=joint_name,
                kp="20",
                kv="9",
                ctrlrange=_vec_to_str((_deg_to_rad(lo), _deg_to_rad(hi))),
            )

    contact = ET.SubElement(root, "contact")
    for i in range(6):
        ET.SubElement(contact, "exclude", name=f"leg{i}_coxa_femur", body1=f"leg{i}_hip", body2=f"leg{i}_femur_body")
        ET.SubElement(contact, "exclude", name=f"leg{i}_femur_tibia", body1=f"leg{i}_femur_body", body2=f"leg{i}_tibia_body")

    return root


def write_mjcf(path: str | Path, timestep: float = 0.01) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    root = build_mjcf_xml(timestep=timestep)
    tree = ET.ElementTree(root)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def main() -> None:
    default_path = Path(__file__).resolve().parent / "models" / "hexapod.xml"
    write_mjcf(default_path)
    print(f"Wrote MJCF to {default_path}")


if __name__ == "__main__":
    main()
