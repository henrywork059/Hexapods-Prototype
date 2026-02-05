# sim_engine

A lightweight **desktop viewer + minimal simulation loop** for the Hexapods-Prototype project.

`sim_engine` is meant to give you **fast visual feedback** while you iterate on gait math, trajectories, and IK—before you send commands to real hardware. The main repo README describes it as a **desktop viewer that renders a simplified 2D physics engine** inside this folder. ([github.com](https://github.com/henrywork059/Hexapods-Prototype/tree/main))

> TL;DR: use this when you want to *see* what your gait/IK is doing, without touching servos.

---

## What it does

- **Runs the desktop viewer** (`python -m sim_engine`) so you can inspect motion and debug gait/trajectory behavior. ([github.com](https://github.com/henrywork059/Hexapods-Prototype/tree/main))
- Provides an optional **settings GUI** that saves a JSON settings file, then launches the viewer. ([github.com](https://github.com/henrywork059/Hexapods-Prototype/tree/main))
- Includes **desktop unit tests** under `sim_engine/tests` (discoverable via `unittest`). ([github.com](https://github.com/henrywork059/Hexapods-Prototype/tree/main))

### What it’s NOT

- Not a full robotics simulator (no ROS/Gazebo, no high-fidelity contact dynamics).
- Not the hardware runtime: real walking on the robot happens in `MCU_Micropython/`.

---

## How to install (VS Code on desktop)

These steps work on Windows / macOS / Linux.

### 1) Prerequisites

- **Python 3.10+** (recommended)
- **Git**
- **VS Code** + the **Python** extension

### 2) Clone the repo and open it in VS Code

```bash
git clone https://github.com/henrywork059/Hexapods-Prototype.git
cd Hexapods-Prototype
code .
```

### 3) Create a virtual environment (recommended)

**Windows (PowerShell)**
```bash
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

**macOS / Linux**
```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 4) Install dependencies

From repo root:

```bash
pip install -r requirements.txt
```

> If `pip` fails, upgrade first:
> `python -m pip install --upgrade pip`

---

## How to run

### Option A — Run the viewer (recommended)

From repo root:

```bash
python -m sim_engine
```

This is the same command referenced by the project’s main README. ([github.com](https://github.com/henrywork059/Hexapods-Prototype/tree/main))

### Option B — Launch settings GUI (then viewer)

From repo root:

```bash
python -c "from sim_engine.viewer_desktop.app import ViewerApp; from sim_engine.viewer_desktop.settings_gui import launch_settings_gui; launch_settings_gui(lambda s: ViewerApp(s).root.mainloop())"
```

This GUI flow (save JSON → start viewer) is also documented in the main README. ([github.com](https://github.com/henrywork059/Hexapods-Prototype/tree/main))

---

## Run tests

From repo root:

```bash
python -m unittest discover sim_engine/tests
```

(As listed in the project README.) ([github.com](https://github.com/henrywork059/Hexapods-Prototype/tree/main))

---

## Troubleshooting

### `ModuleNotFoundError: sim_engine`

- Make sure you run commands **from the repo root** (the folder that contains `sim_engine/`).
- Confirm VS Code selected your `.venv` interpreter:
  - `Ctrl+Shift+P` → **Python: Select Interpreter** → choose `.venv`.

### The window doesn’t open / Tk errors

- The viewer/settings GUI likely use **Tkinter** (common for simple desktop UIs). On some Linux installs, you may need to install Tk:
  - Debian/Ubuntu: `sudo apt-get install python3-tk`

### Tests show `Ran 0 tests`

- Ensure your test files follow `unittest` discovery naming (e.g. `test_*.py`) and folders are importable packages (often requires `__init__.py`).

---

## How it fits the project

- `simulation/` is for **math validation** runs.
- `sim_engine/` is for **desktop visualization** of the simplified engine.
- `MCU_Micropython/` is for **real hardware control**.

If you want the bigger build/validate order for the whole stack (not just `sim_engine`), see `hexapod_project_code_plan_build_validation_order.md`. fileciteturn0file8

