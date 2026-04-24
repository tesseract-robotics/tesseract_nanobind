# Viewer

`tesseract_robotics.viewer` provides a web-based 3D viewer for scenes and trajectories, backed by Three.js. It started life as a standalone package and is now a first-class submodule of `tesseract_robotics`.

## Installation

The viewer ships with the main package. No separate install step.

```python
from tesseract_robotics.viewer import TesseractViewer
```

## Basic Usage

Build an `Environment`, then hand it to the viewer and start the background HTTP server at `http://localhost:8000`:

```python
--8<-- "src/tesseract_robotics/examples/shapes_viewer.py:setup"
```

```python
--8<-- "src/tesseract_robotics/examples/shapes_viewer.py:serve"
```

## Visualizing a Robot

Load a URDF/SRDF pair via `GeneralResourceLocator`, then push the environment into the viewer:

```python
--8<-- "src/tesseract_robotics/examples/abb_irb2400_viewer.py:load_robot"
```

## Animating a Planned Trajectory

After running a motion planner, feed the resulting `CompositeInstruction` to `update_trajectory`. The viewer animates the robot using the timestamps baked into the trajectory:

```python
--8<-- "src/tesseract_robotics/examples/abb_irb2400_viewer.py:plan_and_animate"
```

Then push the result into the viewer (from the same example):

```python
viewer.update_trajectory(final_results_instruction)
viewer.plot_trajectory(final_results_instruction, manip_info, axes_length=0.05)
```

## Industrial Workcells

The Tesseract Robotics Workcell Collection (TWC) ships multi-robot positioner cells. The viewer supports both rail and positioner workcell topologies:

```python
--8<-- "src/tesseract_robotics/examples/twc_workcell_positioner_viewer.py:load_workcell"
```

!!! note "Submodule Required"
    TWC workcell assets ship as a git submodule. Initialize with:
    ```bash
    git submodule update --init
    ```

## API Quick Reference

| Method | Purpose |
|---|---|
| `TesseractViewer(server_address=("127.0.0.1", 8000))` | Create viewer instance; spawns background event loop thread |
| `update_environment(tesseract_env, origin_offset=[0, 0, 0])` | Load or reload a scene |
| `update_joint_positions(joint_names, joint_positions)` | Set stationary joint state (stops any animation) |
| `update_trajectory(tesseract_trajectory)` | Animate a `CompositeInstruction` trajectory |
| `update_trajectory_list(joint_names, trajectory)` | Animate a raw list-of-waypoints trajectory |
| `plot_trajectory(tesseract_trajectory, manipulator_info)` | Draw trajectory path + waypoint axes in the scene |
| `start_serve_background()` | Start HTTP server on `server_address` |
| `close()` | Stop the server and join the background thread |
| `add_axes_marker / add_arrow_marker / add_box_marker / add_sphere_marker / add_cylinder_marker / add_capsule_marker / add_lines_marker` | Add geometric markers to the scene |
| `clear_all_markers / clear_markers_by_tags / clear_markers_by_name` | Remove markers |
| `save(directory)` / `save_scene_gltf(fname)` / `save_scene_glb(fname)` | Export the scene for offline viewing |

For the full API, see [`tesseract_robotics.viewer` source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/viewer/tesseract_viewer.py).

## Examples

- [`shapes_viewer.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/shapes_viewer.py) — primitives
- [`tesseract_material_mesh_viewer.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/tesseract_material_mesh_viewer.py) — materials + meshes
- [`abb_irb2400_viewer.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/abb_irb2400_viewer.py) — robot + planned trajectory
- [`twc_workcell_positioner_viewer.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/twc_workcell_positioner_viewer.py) — industrial workcell (requires `git submodule update --init`)
