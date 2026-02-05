"""Tkinter-based viewer for the simulation engine."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import tkinter as tk
from tkinter import messagebox, ttk

from sim_engine import config
from sim_engine.engine.collision import detect_contacts
from sim_engine.engine.materials import Material
from sim_engine.engine.rigid_body import BodyType, RigidBody, Vec2
from sim_engine.engine.shapes import Box, Circle
from sim_engine.engine.world import World
from sim_engine.replay.recorder import FrameSnapshot, ReplayRecorder, load_recording, save_recording
from sim_engine.settings_schema import SimSettings, load_last_used

from .camera import Camera2D
from .input import InputState
from .overlays import OverlayColors, draw_body, draw_contacts, draw_ground, draw_velocity_vectors


@dataclass
class SceneSpec:
    name: str
    builder: Callable[[World, SimSettings], None]


def _material_from_settings(settings: SimSettings) -> Material:
    return Material(friction=settings.mu_dynamic, restitution=settings.restitution)


def _spawn_box(world: World, settings: SimSettings, position: Vec2, size: tuple[float, float], angle: float = 0.0) -> None:
    material = _material_from_settings(settings)
    body = RigidBody(
        shape=Box(size[0] * 0.5, size[1] * 0.5),
        body_type=BodyType.DYNAMIC,
        position=position,
        angle=angle,
        material=material,
    )
    world.add_body(body)


def _spawn_circle(world: World, settings: SimSettings, position: Vec2, radius: float) -> None:
    material = _material_from_settings(settings)
    body = RigidBody(
        shape=Circle(radius),
        body_type=BodyType.DYNAMIC,
        position=position,
        material=material,
    )
    world.add_body(body)


def _build_default_scene(world: World, settings: SimSettings) -> None:
    _spawn_box(world, settings, Vec2(-120.0, 80.0), (60.0, 40.0), angle=math.radians(15))
    _spawn_box(world, settings, Vec2(90.0, 140.0), (80.0, 30.0), angle=math.radians(-10))
    _spawn_circle(world, settings, Vec2(0.0, 200.0), 25.0)
    _spawn_circle(world, settings, Vec2(140.0, 260.0), 20.0)


def _build_stack_scene(world: World, settings: SimSettings) -> None:
    base_x = -80.0
    for i in range(6):
        _spawn_box(
            world,
            settings,
            Vec2(base_x + i * 30.0, 40.0 + i * 35.0),
            (50.0, 20.0),
            angle=math.radians(2 * i),
        )
    for i in range(4):
        _spawn_circle(world, settings, Vec2(80.0 + i * 35.0, 220.0 + i * 40.0), 18.0)


def _build_drop_scene(world: World, settings: SimSettings) -> None:
    for i in range(10):
        x = world.rng.uniform(-200.0, 200.0)
        z = world.rng.uniform(120.0, 320.0)
        if i % 2 == 0:
            _spawn_circle(world, settings, Vec2(x, z), world.rng.uniform(12.0, 24.0))
        else:
            size = world.rng.uniform(20.0, 50.0)
            _spawn_box(world, settings, Vec2(x, z), (size, size * 0.6), angle=world.rng.uniform(-0.5, 0.5))


SCENES = [
    SceneSpec("default", _build_default_scene),
    SceneSpec("stack", _build_stack_scene),
    SceneSpec("drop", _build_drop_scene),
]


class ViewerApp:
    def __init__(self, settings: SimSettings) -> None:
        self.settings = settings
        self.root = tk.Tk()
        self.root.title("Sim Engine Viewer")
        self.root.geometry(f"{settings.window_width}x{settings.window_height}")

        self._build_layout()
        self.root.update_idletasks()

        self.world = self._create_world(settings)
        self.scene_name = self._resolve_scene_name(settings.scene)
        self.scene_combo.set(self.scene_name)

        self.camera = Camera2D(
            width=settings.window_width,
            height=self.canvas.winfo_height() or settings.window_height,
            zoom=settings.zoom,
            pixels_per_mm=2.0,
        )
        self.input_state = InputState(self.camera)

        self.paused = False
        self.step_requested = False
        self.last_time = time.perf_counter()
        self.sim_time = 0.0
        self.colors = OverlayColors()
        self.latest_contact_count = 0
        self.replay: list[FrameSnapshot] | None = None
        self.replay_index = 0
        self.replay_time = 0.0
        self.replay_contact_count = 0
        self.recorder: ReplayRecorder | None = None
        self.playback_enabled = settings.playback_enabled

        self._configure_replay_or_recording()
        self._bind_events()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._loop()

    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        self.toolbar = ttk.Frame(self.root, padding=4)
        self.toolbar.grid(row=0, column=0, sticky="ew")
        self.toolbar.columnconfigure(10, weight=1)

        self.play_button = ttk.Button(self.toolbar, text="Pause", command=self._toggle_pause)
        self.play_button.grid(row=0, column=0, padx=4)

        ttk.Button(self.toolbar, text="Step", command=self._request_step).grid(row=0, column=1, padx=4)
        ttk.Button(self.toolbar, text="Reset", command=self._reset_scene).grid(row=0, column=2, padx=4)

        ttk.Label(self.toolbar, text="Scene").grid(row=0, column=3, padx=(12, 4))
        self.scene_combo = ttk.Combobox(
            self.toolbar,
            values=[scene.name for scene in SCENES],
            state="readonly",
            width=12,
        )
        self.scene_combo.bind("<<ComboboxSelected>>", self._on_scene_selected)
        self.scene_combo.grid(row=0, column=4, padx=4)

        self.spawn_box_button = ttk.Button(self.toolbar, text="Spawn Box", command=self._spawn_random_box)
        self.spawn_box_button.grid(row=0, column=5, padx=4)
        self.spawn_circle_button = ttk.Button(self.toolbar, text="Spawn Circle", command=self._spawn_random_circle)
        self.spawn_circle_button.grid(row=0, column=6, padx=4)

        self.stats_label = ttk.Label(self.toolbar, text="")
        self.stats_label.grid(row=0, column=11, sticky="e")

        self.canvas = tk.Canvas(self.root, bg="#F7F7F7", highlightthickness=0)
        self.canvas.grid(row=1, column=0, sticky="nsew")

    def _bind_events(self) -> None:
        self.canvas.bind("<ButtonPress-1>", self._on_mouse_down)
        self.canvas.bind("<ButtonRelease-1>", self._on_mouse_up)
        self.canvas.bind("<B1-Motion>", self._on_mouse_drag)
        self.canvas.bind("<MouseWheel>", self._on_mouse_wheel)
        self.canvas.bind("<Button-4>", self._on_mouse_wheel)
        self.canvas.bind("<Button-5>", self._on_mouse_wheel)
        self.canvas.bind("<Configure>", self._on_resize)
        self.root.bind("<space>", lambda _event: self._toggle_pause())
        self.root.bind("<KeyPress-s>", lambda _event: self._request_step())
        self.root.bind("<KeyPress-r>", lambda _event: self._reset_scene())

    def _configure_replay_or_recording(self) -> None:
        if self.playback_enabled:
            try:
                recording = load_recording(Path(self.settings.playback_path))
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                messagebox.showerror("Playback Load", f"Unable to load replay: {exc}")
                self.playback_enabled = False
            else:
                self.replay = recording.frames
                if not self.replay:
                    messagebox.showerror("Playback Load", "Replay file contains no frames.")
                    self.playback_enabled = False

        if self.playback_enabled and self.replay:
            self._apply_replay_frame(self.replay[0])
            self.replay_index = 0
            self.replay_time = self.replay[0].time
            self.sim_time = self.replay[0].time
            self.scene_combo.configure(state="disabled")
            self.spawn_box_button.configure(state="disabled")
            self.spawn_circle_button.configure(state="disabled")
        else:
            self._build_scene(self.scene_name)
            if self.settings.recording_enabled:
                self.recorder = ReplayRecorder(
                    fps=self.settings.recording_fps,
                    metadata={
                        "scene": self.scene_name,
                        "recording_fps": self.settings.recording_fps,
                    },
                )

    def _create_world(self, settings: SimSettings) -> World:
        world = World(
            fixed_dt=settings.fixed_dt,
            gravity=Vec2(settings.gravity_x, settings.gravity_z),
            solver_iterations=settings.solver_iterations,
            ground_z=settings.ground_z,
            seed=settings.seed,
        )
        return world

    def _build_scene(self, scene_name: str) -> None:
        self.world.bodies.clear()
        scene = next((scene for scene in SCENES if scene.name == scene_name), SCENES[0])
        scene.builder(self.world, self.settings)

    @staticmethod
    def _resolve_scene_name(scene_name: str) -> str:
        return scene_name if any(scene.name == scene_name for scene in SCENES) else SCENES[0].name

    def _toggle_pause(self) -> None:
        self.paused = not self.paused
        self.play_button.configure(text="Play" if self.paused else "Pause")

    def _request_step(self) -> None:
        self.step_requested = True

    def _reset_scene(self) -> None:
        self.sim_time = 0.0
        if self.playback_enabled and self.replay:
            self.replay_index = 0
            self.replay_time = self.replay[0].time
            self._apply_replay_frame(self.replay[0])
            self.sim_time = self.replay[0].time
        else:
            self.world = self._create_world(self.settings)
            self._build_scene(self.scene_name)
            if self.recorder:
                self.recorder.reset()

    def _on_scene_selected(self, _event: tk.Event) -> None:
        self.scene_name = self.scene_combo.get() or self.scene_name
        self._reset_scene()

    def _spawn_random_box(self) -> None:
        x = self.world.rng.uniform(-200.0, 200.0)
        z = self.world.rng.uniform(180.0, 320.0)
        width = self.world.rng.uniform(30.0, 70.0)
        height = self.world.rng.uniform(20.0, 60.0)
        angle = self.world.rng.uniform(-0.4, 0.4)
        _spawn_box(self.world, self.settings, Vec2(x, z), (width, height), angle=angle)

    def _spawn_random_circle(self) -> None:
        x = self.world.rng.uniform(-200.0, 200.0)
        z = self.world.rng.uniform(180.0, 320.0)
        radius = self.world.rng.uniform(15.0, 30.0)
        _spawn_circle(self.world, self.settings, Vec2(x, z), radius)

    def _on_mouse_down(self, event: tk.Event) -> None:
        self.input_state.start_pan(event.x, event.y)

    def _on_mouse_up(self, _event: tk.Event) -> None:
        self.input_state.end_pan()

    def _on_mouse_drag(self, event: tk.Event) -> None:
        self.input_state.pan_to(event.x, event.y)

    def _on_mouse_wheel(self, event: tk.Event) -> None:
        if hasattr(event, "delta") and event.delta:
            factor = 1.1 if event.delta > 0 else 0.9
        elif getattr(event, "num", None) == 4:
            factor = 1.1
        else:
            factor = 0.9
        self.input_state.zoom(factor, event.x, event.y)

    def _on_resize(self, event: tk.Event) -> None:
        self.camera.resize(event.width, event.height)

    def _loop(self) -> None:
        now = time.perf_counter()
        dt = now - self.last_time
        self.last_time = now
        advanced = False
        if self.playback_enabled and self.replay:
            if not self.paused:
                advanced = self._advance_playback(dt)
            elif self.step_requested:
                advanced = self._advance_playback(dt, single_step=True)
                self.step_requested = False
            else:
                self.step_requested = False
        else:
            if not self.paused:
                self.world.step(dt)
                self.sim_time += dt
                advanced = True
            elif self.step_requested:
                self.world.step(self.settings.fixed_dt)
                self.sim_time += self.settings.fixed_dt
                self.step_requested = False
                advanced = True
            else:
                self.step_requested = False

        contacts = self._compute_contacts()
        if advanced:
            self._record_frame()
        self._render(contacts)
        self.root.after(int(1000 / self.settings.fps), self._loop)

    def _render(self, contacts: list) -> None:
        self.canvas.delete("all")
        draw_ground(self.canvas, self.camera, self.settings.ground_z, self.camera.width)
        for body in self.world.bodies:
            draw_body(self.canvas, self.camera, body, self.colors)
        if self.settings.show_contacts:
            draw_contacts(self.canvas, self.camera, contacts, self.colors)
        draw_velocity_vectors(self.canvas, self.camera, self.world.bodies, self.colors)
        self.stats_label.configure(
            text=(
                f"Bodies: {len(self.world.bodies)}  "
                f"Contacts: {self.latest_contact_count}  "
                f"Time: {self.sim_time:0.2f}s  "
                f"Zoom: {self.camera.zoom:0.2f}"
            )
        )

    def _compute_contacts(self) -> list:
        if self.playback_enabled and self.replay:
            self.latest_contact_count = self.replay_contact_count
            return []
        contacts = detect_contacts(self.world.bodies, self.settings.ground_z, self.world.ground_body)
        self.latest_contact_count = len(contacts)
        return contacts if self.settings.show_contacts else []

    def _record_frame(self) -> None:
        if self.recorder is None:
            return
        self.recorder.capture(self.world.bodies, self.sim_time, self.latest_contact_count)

    def _apply_replay_frame(self, frame: FrameSnapshot) -> None:
        self.world.bodies = [body.to_body() for body in frame.bodies]
        self.replay_contact_count = frame.contact_count

    def _advance_playback(self, dt: float, single_step: bool = False) -> bool:
        if not self.replay:
            return False
        if single_step:
            if self.replay_index + 1 < len(self.replay):
                self.replay_index += 1
            frame = self.replay[self.replay_index]
            self._apply_replay_frame(frame)
            self.replay_time = frame.time
            self.sim_time = frame.time
            return True
        self.replay_time += dt
        while self.replay_index + 1 < len(self.replay) and self.replay[self.replay_index + 1].time <= self.replay_time:
            self.replay_index += 1
        frame = self.replay[self.replay_index]
        self._apply_replay_frame(frame)
        self.sim_time = frame.time
        return True

    def _on_close(self) -> None:
        if self.recorder and self.recorder.frames:
            recording = self.recorder.build_recording()
            try:
                save_recording(recording, Path(self.settings.recording_path))
            except OSError as exc:
                messagebox.showerror("Recording Save", f"Unable to save recording: {exc}")
        self.root.destroy()


def main() -> None:
    settings = load_last_used()
    app = ViewerApp(settings)
    app.root.mainloop()


if __name__ == "__main__":
    main()
