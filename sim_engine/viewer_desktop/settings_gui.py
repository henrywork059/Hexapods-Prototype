"""Tkinter settings GUI for the desktop simulation viewer."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Callable

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from sim_engine import config
from sim_engine.settings_schema import SimSettings, load_last_used, save_last_used


class SettingsGUI:
    """Render a settings form and emit SimSettings on start."""

    def __init__(self, on_start: Callable[[SimSettings], None]):
        self._on_start = on_start
        self.root = tk.Tk()
        self.root.title("Simulation Settings")

        self.settings_path_var = tk.StringVar(value=str(config.DEFAULT_SETTINGS_PATH))

        self.scene_var = tk.StringVar(value=config.DEFAULT_SCENE)
        self.fixed_dt_var = tk.StringVar(value=str(config.DEFAULT_FIXED_DT))
        self.fps_var = tk.StringVar(value=str(config.DEFAULT_FPS))
        self.gravity_x_var = tk.StringVar(value=str(config.DEFAULT_GRAVITY_X))
        self.gravity_z_var = tk.StringVar(value=str(config.DEFAULT_GRAVITY_Z))
        self.solver_iterations_var = tk.StringVar(value=str(config.DEFAULT_SOLVER_ITERATIONS))
        self.mu_static_var = tk.StringVar(value=str(config.DEFAULT_MU_STATIC))
        self.mu_dynamic_var = tk.StringVar(value=str(config.DEFAULT_MU_DYNAMIC))
        self.restitution_var = tk.StringVar(value=str(config.DEFAULT_RESTITUTION))
        self.seed_var = tk.StringVar(value="" if config.DEFAULT_SEED is None else str(config.DEFAULT_SEED))
        self.window_width_var = tk.StringVar(value=str(config.DEFAULT_WINDOW_WIDTH))
        self.window_height_var = tk.StringVar(value=str(config.DEFAULT_WINDOW_HEIGHT))
        self.zoom_var = tk.StringVar(value=str(config.DEFAULT_ZOOM))

        self.show_axes_var = tk.BooleanVar(value=config.DEFAULT_SHOW_AXES)
        self.show_contacts_var = tk.BooleanVar(value=config.DEFAULT_SHOW_CONTACTS)
        self.show_stats_var = tk.BooleanVar(value=config.DEFAULT_SHOW_STATS)

        self.recording_enabled_var = tk.BooleanVar(value=config.DEFAULT_RECORDING_ENABLED)
        self.recording_path_var = tk.StringVar(value=str(config.DEFAULT_RECORDING_PATH))
        self.recording_fps_var = tk.StringVar(value=str(config.DEFAULT_RECORDING_FPS))
        self.playback_enabled_var = tk.BooleanVar(value=config.DEFAULT_PLAYBACK_ENABLED)
        self.playback_path_var = tk.StringVar(value=str(config.DEFAULT_PLAYBACK_PATH))

        self._build_layout()
        self._load_from_settings(load_last_used())

    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1)

        settings_frame = ttk.Frame(self.root, padding=12)
        settings_frame.grid(row=0, column=0, sticky="nsew")
        settings_frame.columnconfigure(0, weight=1)

        file_frame = ttk.LabelFrame(settings_frame, text="Settings File", padding=8)
        file_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        file_frame.columnconfigure(1, weight=1)

        ttk.Label(file_frame, text="Path").grid(row=0, column=0, sticky="w")
        ttk.Entry(file_frame, textvariable=self.settings_path_var).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(file_frame, text="Browse", command=self._browse_settings_path).grid(row=0, column=2, padx=4)
        ttk.Button(file_frame, text="Load", command=self._load_from_path).grid(row=0, column=3, padx=4)
        ttk.Button(file_frame, text="Save", command=self._save_to_path).grid(row=0, column=4, padx=4)

        sim_frame = ttk.LabelFrame(settings_frame, text="Simulation", padding=8)
        sim_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        sim_frame.columnconfigure(1, weight=1)
        self._add_row(sim_frame, 0, "Scene", self.scene_var)
        self._add_row(sim_frame, 1, "Fixed dt", self.fixed_dt_var)
        self._add_row(sim_frame, 2, "FPS", self.fps_var)
        self._add_row(sim_frame, 3, "Gravity X", self.gravity_x_var)
        self._add_row(sim_frame, 4, "Gravity Z", self.gravity_z_var)
        self._add_row(sim_frame, 5, "Solver iterations", self.solver_iterations_var)
        self._add_row(sim_frame, 6, "Mu static", self.mu_static_var)
        self._add_row(sim_frame, 7, "Mu dynamic", self.mu_dynamic_var)
        self._add_row(sim_frame, 8, "Restitution", self.restitution_var)
        self._add_row(sim_frame, 9, "Seed", self.seed_var)

        view_frame = ttk.LabelFrame(settings_frame, text="View", padding=8)
        view_frame.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        view_frame.columnconfigure(1, weight=1)
        self._add_row(view_frame, 0, "Window width", self.window_width_var)
        self._add_row(view_frame, 1, "Window height", self.window_height_var)
        self._add_row(view_frame, 2, "Zoom", self.zoom_var)

        overlay_frame = ttk.LabelFrame(settings_frame, text="Overlays", padding=8)
        overlay_frame.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        ttk.Checkbutton(overlay_frame, text="Show axes", variable=self.show_axes_var).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(overlay_frame, text="Show contacts", variable=self.show_contacts_var).grid(
            row=0, column=1, sticky="w", padx=12
        )
        ttk.Checkbutton(overlay_frame, text="Show stats", variable=self.show_stats_var).grid(
            row=0, column=2, sticky="w"
        )

        recording_frame = ttk.LabelFrame(settings_frame, text="Recording", padding=8)
        recording_frame.grid(row=4, column=0, sticky="ew", pady=(0, 10))
        recording_frame.columnconfigure(1, weight=1)
        ttk.Checkbutton(recording_frame, text="Enable recording", variable=self.recording_enabled_var).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(recording_frame, text="Path").grid(row=1, column=0, sticky="w")
        ttk.Entry(recording_frame, textvariable=self.recording_path_var).grid(row=1, column=1, sticky="ew", padx=6)
        ttk.Button(recording_frame, text="Browse", command=self._browse_recording_path).grid(row=1, column=2)
        self._add_row(recording_frame, 2, "Recording FPS", self.recording_fps_var)

        playback_frame = ttk.LabelFrame(settings_frame, text="Playback", padding=8)
        playback_frame.grid(row=5, column=0, sticky="ew", pady=(0, 10))
        playback_frame.columnconfigure(1, weight=1)
        ttk.Checkbutton(playback_frame, text="Enable playback", variable=self.playback_enabled_var).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(playback_frame, text="Replay path").grid(row=1, column=0, sticky="w")
        ttk.Entry(playback_frame, textvariable=self.playback_path_var).grid(row=1, column=1, sticky="ew", padx=6)
        ttk.Button(playback_frame, text="Browse", command=self._browse_playback_path).grid(row=1, column=2)

        button_frame = ttk.Frame(settings_frame)
        button_frame.grid(row=6, column=0, sticky="e")
        ttk.Button(button_frame, text="Start Simulation", command=self._start_simulation).grid(row=0, column=0)

    @staticmethod
    def _add_row(parent: ttk.Widget, row: int, label: str, variable: tk.StringVar) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Entry(parent, textvariable=variable).grid(row=row, column=1, sticky="ew", pady=2)

    def _browse_settings_path(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Select settings JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            defaultextension=".json",
        )
        if path:
            self.settings_path_var.set(path)

    def _browse_recording_path(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Select recording path",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            defaultextension=".json",
        )
        if path:
            self.recording_path_var.set(path)

    def _browse_playback_path(self) -> None:
        path = filedialog.askopenfilename(
            title="Select replay JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if path:
            self.playback_path_var.set(path)

    def _load_from_settings(self, settings: SimSettings) -> None:
        self.scene_var.set(settings.scene)
        self.fixed_dt_var.set(str(settings.fixed_dt))
        self.fps_var.set(str(settings.fps))
        self.gravity_x_var.set(str(settings.gravity_x))
        self.gravity_z_var.set(str(settings.gravity_z))
        self.solver_iterations_var.set(str(settings.solver_iterations))
        self.mu_static_var.set(str(settings.mu_static))
        self.mu_dynamic_var.set(str(settings.mu_dynamic))
        self.restitution_var.set(str(settings.restitution))
        self.seed_var.set("" if settings.seed is None else str(settings.seed))
        self.window_width_var.set(str(settings.window_width))
        self.window_height_var.set(str(settings.window_height))
        self.zoom_var.set(str(settings.zoom))
        self.show_axes_var.set(settings.show_axes)
        self.show_contacts_var.set(settings.show_contacts)
        self.show_stats_var.set(settings.show_stats)
        self.recording_enabled_var.set(settings.recording_enabled)
        self.recording_path_var.set(settings.recording_path)
        self.recording_fps_var.set(str(settings.recording_fps))
        self.playback_enabled_var.set(settings.playback_enabled)
        self.playback_path_var.set(settings.playback_path)

    def _load_from_path(self) -> None:
        path = self._resolve_settings_path()
        if path is None:
            return
        try:
            payload = json.loads(Path(path).read_text())
        except FileNotFoundError:
            messagebox.showerror("Settings Load", f"Settings file not found: {path}")
            return
        except json.JSONDecodeError as exc:
            messagebox.showerror("Settings Load", f"Invalid JSON: {exc}")
            return
        settings = SimSettings.from_json(payload if isinstance(payload, dict) else {})
        self._load_from_settings(settings)

    def _save_to_path(self) -> None:
        settings = self._collect_settings()
        if settings is None:
            return
        path = self._resolve_settings_path()
        if path is None:
            return
        save_last_used(settings, Path(path))
        messagebox.showinfo("Settings Save", f"Saved settings to {path}")

    def _resolve_settings_path(self) -> str | None:
        path = self.settings_path_var.get().strip()
        if path:
            return path
        selected = filedialog.asksaveasfilename(
            title="Select settings JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            defaultextension=".json",
        )
        if selected:
            self.settings_path_var.set(selected)
        return selected or None

    def _collect_settings(self) -> SimSettings | None:
        try:
            seed_value = self.seed_var.get().strip()
            seed = int(seed_value) if seed_value else None
            return SimSettings(
                scene=self.scene_var.get().strip() or config.DEFAULT_SCENE,
                fixed_dt=float(self.fixed_dt_var.get()),
                fps=float(self.fps_var.get()),
                gravity_x=float(self.gravity_x_var.get()),
                gravity_z=float(self.gravity_z_var.get()),
                solver_iterations=int(self.solver_iterations_var.get()),
                mu_static=float(self.mu_static_var.get()),
                mu_dynamic=float(self.mu_dynamic_var.get()),
                restitution=float(self.restitution_var.get()),
                seed=seed,
                window_width=int(self.window_width_var.get()),
                window_height=int(self.window_height_var.get()),
                zoom=float(self.zoom_var.get()),
                show_axes=self.show_axes_var.get(),
                show_contacts=self.show_contacts_var.get(),
                show_stats=self.show_stats_var.get(),
                recording_enabled=self.recording_enabled_var.get(),
                recording_path=self.recording_path_var.get().strip() or config.DEFAULT_RECORDING_PATH,
                recording_fps=float(self.recording_fps_var.get()),
                playback_enabled=self.playback_enabled_var.get(),
                playback_path=self.playback_path_var.get().strip() or config.DEFAULT_PLAYBACK_PATH,
            )
        except ValueError as exc:
            messagebox.showerror("Invalid settings", f"Please check the input values.\n\n{exc}")
            return None

    def _start_simulation(self) -> None:
        settings = self._collect_settings()
        if settings is None:
            return
        save_last_used(settings)
        self._on_start(settings)

    def run(self) -> None:
        self.root.mainloop()


def launch_settings_gui(on_start: Callable[[SimSettings], None]) -> None:
    """Convenience entry-point for the settings GUI."""

    gui = SettingsGUI(on_start=on_start)
    gui.run()
