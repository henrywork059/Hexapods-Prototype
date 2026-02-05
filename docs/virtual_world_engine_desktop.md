# Virtual World Engine (Desktop) Guide

This document summarizes the current **desktop simulation engine** in `sim_engine/`, including architecture, API surfaces, solver math, units, viewer controls, and the settings GUI.

## Architecture overview

The desktop engine is split into a physics core, demo/scene builders, and a Tkinter-based viewer.

### 1) Physics core (`sim_engine/engine`)

* **World** (`World`): manages bodies, integration, contact detection, and the solver loop. It accumulates time and steps at a fixed `fixed_dt`, applying gravity, forces, and impulses each step. It also hosts a ground body and a seeded RNG for deterministic scenes. (`engine/world.py`)
* **Rigid bodies** (`RigidBody`, `BodyType`, `Vec2`): 2D rigid bodies living in the X/Z plane, supporting static/dynamic/kinematic bodies, forces, impulses, and per-body material properties. (`engine/rigid_body.py`)
* **Shapes** (`Circle`, `Box`): geometric primitives with moment-of-inertia calculations for integration. (`engine/shapes.py`)
* **Collision & contacts**: broad logic for circle/box collisions and ground plane collision, with contact points, normals, and penetration depth. (`engine/collision.py`, `engine/contacts.py`)
* **Materials**: friction and restitution with combination rules (sqrt for friction, max for restitution). (`engine/materials.py`)
* **Solver** (`SequentialImpulseSolver`): iterative impulse resolution for contacts, plus positional correction. (`engine/solver.py`)

### 2) Scene builders (`sim_engine/demos`)

Scene helpers create reusable demo setups and return optional per-step update callbacks. The settings GUI uses these scenes by name. (`demos/scenes.py`)

### 3) Desktop viewer (`sim_engine/viewer_desktop`)

* **Viewer app**: Tkinter canvas for rendering bodies and overlays; toolbar actions for pause, step, reset, and spawning bodies. (`viewer_desktop/app.py`)
* **Camera & input**: 2D camera for pan/zoom and conversion between world and screen coordinates. (`viewer_desktop/camera.py`, `viewer_desktop/input.py`)
* **Overlays**: rendering of ground, bodies, contacts, and velocity vectors. (`viewer_desktop/overlays.py`)
* **Settings GUI**: a Tkinter form for configuring simulation parameters, view, overlays, and replay options. (`viewer_desktop/settings_gui.py`)
* **Replay/recording**: JSON recording helpers for capture and playback. (`replay/recorder.py`)

## API quick reference

This is a practical cheat-sheet for building scenes or plugging into the engine.

### Core types

* `Vec2(x, z)` — 2D vector in world space (X/Z).
* `RigidBody(shape, body_type, position, angle, ...)` — body with velocity, angular velocity, force, torque, and material. Use `apply_force` or `apply_impulse` to update.
* `BodyType.STATIC | DYNAMIC | KINEMATIC` — static and kinematic bodies ignore forces; dynamic bodies simulate forces/impulses.
* `Circle(radius)` and `Box(half_width, half_height)` — supported shapes.

### World

* `World.add_body(body)` — add a body to the simulation.
* `World.step(dt, on_step=None, on_pre_step=None)` — advance the simulation using fixed timesteps; optional hooks run before/after each fixed step.
* `World.fixed_dt`, `gravity`, `solver_iterations`, `ground_z`, `seed` — primary runtime properties.

### Materials & contacts

* `Material(friction, restitution)` — per-body material.
* `Contact(point, normal, penetration)` — produced during collision detection and consumed by the solver.

## Solver math (Sequential Impulse)

The solver applies impulses per-contact to resolve interpenetration and simulate restitution/friction:

1. **Relative velocity** at the contact point is computed using linear and angular velocity.
2. **Normal impulse** `j` is applied to separate bodies based on restitution:
   * `j = -(1 + e) * v_rel · n / (inv_mass_sum + rotational_terms)`
3. **Friction impulse** `jt` uses a tangent direction and is clamped by `μ * j`:
   * `jt = clamp(-v_rel · t / inv_mass_sum, -μj, +μj)`
4. **Positional correction** (Baumgarte-style) nudges bodies apart if penetration exceeds a small slop value.

The solver iterates multiple times (`solver_iterations`) for stability. (`engine/solver.py`)

## Units, axes, and coordinate system

* **Units**: millimeters (distance), seconds (time), radians (angles), kilograms (mass).
* **Axes**: the engine is 2D in the **X/Z plane**.
  * `x` = horizontal axis
  * `z` = vertical axis (gravity is negative Z by default)
* **Gravity** defaults to `-9810 mm/s^2` (roughly Earth gravity in mm units). (`config.py`)
* **Ground plane** is at `ground_z` (default `0.0`).

## Desktop viewer controls

The viewer is in `sim_engine/viewer_desktop/app.py` and supports the following controls:

### Toolbar actions

* **Pause / Play** — toggles the simulation loop.
* **Step** — advances by a single fixed timestep.
* **Reset** — recreates the scene from scratch.
* **Scene** — select a scene (default/stack/drop in the viewer app).
* **Spawn Box / Spawn Circle** — adds a random body.

### Keyboard shortcuts

* **Space** — toggle Pause/Play
* **S** — single step
* **R** — reset scene

### Mouse controls

* **Left-drag** — pan camera
* **Mouse wheel** — zoom in/out

## Settings GUI

The settings form in `viewer_desktop/settings_gui.py` lets you edit and persist runtime settings:

### Settings file panel

* Load/save JSON settings via a configurable path (default `~/.sim_engine_settings.json`).

### Simulation section

* Scene selection (uses the scene list from `sim_engine/demos/scenes.py`).
* Fixed timestep, FPS, gravity (X/Z), solver iterations, friction/restitution, RNG seed.

### View section

* Window size and zoom defaults for the viewer window.

### Overlays section

* Toggle axes, contact markers, and stats overlay flags.

### Recording/Playback

* Enable recording and define the JSON path + recording FPS.
* Enable playback and provide a replay JSON to load on startup.

## Related files

* `docs/demos_desktop.md` — desktop demo scene descriptions.
* `docs/CHANGELOG_sim_engine.md` — sim engine changelog.
