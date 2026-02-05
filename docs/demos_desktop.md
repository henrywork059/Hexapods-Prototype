# Desktop Demos (Scenes)

This document lists the current desktop scenes and what you should expect to see when running them.

## Viewer app scenes (`sim_engine/viewer_desktop/app.py`)

These are the scenes in the **toolbar scene dropdown** of the desktop viewer.

### 1) `default`

* **Setup**: a small mix of two angled boxes and two circles with different spawn heights.
* **Expected behavior**: objects fall under gravity, collide with the ground, and settle. Mild rotation should occur due to the initial angle on boxes.

### 2) `stack`

* **Setup**: staggered boxes with increasing height/angle, plus a few circles above.
* **Expected behavior**: the stack should wobble as the upper bodies drop; collisions should cause light rearrangement.

### 3) `drop`

* **Setup**: randomized boxes/circles dropped from a height within a wide X range.
* **Expected behavior**: bodies drop at varied positions, collide with the ground, and spread out as they settle.

## Settings GUI scenes (`sim_engine/demos/scenes.py`)

These are the scenes available in the **settings GUI** (and intended for deterministic demos or tuning).

### 1) `falling_settle`

* **Setup**: alternating boxes and circles spawned across the ground at increasing heights.
* **Expected behavior**: mixed shapes fall and settle; good for baseline stability and contact resolution.

### 2) `friction_slide`

* **Setup**: a box and circle launched with high horizontal velocity across the ground.
* **Expected behavior**: the bodies slide and decelerate; higher friction values shorten the travel distance.

### 3) `stack_collision`

* **Setup**: a stack of boxes plus a fast-moving circle aimed at the stack.
* **Expected behavior**: the moving circle knocks the stack; good for testing restitution and impulse response.

### 4) `hexapod_feet`

* **Setup**: six kinematic circles act as placeholder “feet,” moving in an alternating sinusoidal pattern.
* **Expected behavior**: feet trace a looping stride with lift; useful for visualizing periodic kinematic motion.

## Replay behavior notes

* If **playback** is enabled, the viewer loads a recorded JSON and disables scene selection + spawning, rendering the recorded body states instead.
* If **recording** is enabled, the viewer captures frame snapshots at the configured recording FPS and saves on exit.
