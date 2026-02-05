"""Logging helpers for MuJoCo simulations."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable

import mujoco

from . import bridge


class MjLogger:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        path: str | Path,
        log_sites: bool = True,
    ) -> None:
        self.model = model
        self.data = data
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

        self.joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in bridge.JOINT_ORDER]
        self.joint_qpos = [model.jnt_qposadr[jid] for jid in self.joint_ids]

        if log_sites:
            self.site_ids = []
            for i in range(6):
                name = f"foot_leg{i}"
                sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
                self.site_ids.append(sid)
        else:
            self.site_ids = []

        self._file = self.path.open("w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self._header())

    def _header(self) -> list[str]:
        header = [
            "time",
            "torso_x",
            "torso_y",
            "torso_z",
            "torso_qw",
            "torso_qx",
            "torso_qy",
            "torso_qz",
        ]
        header.extend([f"target_{name}" for name in bridge.JOINT_ORDER])
        header.extend([f"joint_{name}" for name in bridge.JOINT_ORDER])
        for i in range(6):
            header.extend([f"foot{i}_x", f"foot{i}_y", f"foot{i}_z"])
        return header

    def log(self) -> None:
        qpos = self.data.qpos
        row = [
            float(self.data.time),
            float(qpos[0]),
            float(qpos[1]),
            float(qpos[2]),
            float(qpos[3]),
            float(qpos[4]),
            float(qpos[5]),
            float(qpos[6]),
        ]
        row.extend(float(v) for v in self.data.ctrl)
        row.extend(float(qpos[idx]) for idx in self.joint_qpos)

        if self.site_ids:
            for sid in self.site_ids:
                pos = self.data.site_xpos[sid]
                row.extend([float(pos[0]), float(pos[1]), float(pos[2])])
        else:
            row.extend(["", "", ""] * 6)

        self._writer.writerow(row)

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
