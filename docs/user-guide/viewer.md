# Viewer

`tesseract_robotics.viewer` draws a tesseract environment in your browser and animates planned trajectories. It is built on [viser](https://viser.studio). Tesseract owns the kinematics: every link is one viser frame posed from `env.getState(...)`, so rails, positioners and attached objects need no separate handling. After a command that adds or removes links, call `update_environment` again; until then pose updates raise `StaleSceneError`.

## Basic Usage

Build an `Environment`, then hand it to the viewer. The viewer serves from construction until `close()`, at `http://127.0.0.1:8000` by default:

```python
--8<-- "src/tesseract_robotics/examples/shapes_viewer.py:setup"
```

```python
--8<-- "src/tesseract_robotics/examples/shapes_viewer.py:serve"
```

`TesseractViewer(port=...)` picks another port; `port=0` lets the OS choose and `viewer.url` reports it. A taken port raises `ViewerPortInUseError` instead of moving on, because a URL you were given should stay true. `TesseractViewer(host="0.0.0.0")` exposes the viewer to your network; it has no authentication.

## Visualizing a Robot

Load a URDF/SRDF pair via `GeneralResourceLocator`, then push the environment into the viewer:

```python
--8<-- "src/tesseract_robotics/examples/abb_irb2400_viewer.py:load_robot"
```

## Animating a Planned Trajectory

After running a motion planner, feed the resulting `CompositeInstruction` to `update_trajectory`:

```python
--8<-- "src/tesseract_robotics/examples/abb_irb2400_viewer.py:plan_and_animate"
```

Then push the result into the viewer (from the same example):

```python
viewer.update_environment(results["env"])
viewer.update_trajectory(results["trajectory"])
viewer.plot_trajectory(results["trajectory"], results["manip_info"], axes_length=0.05)
```

Waypoint times decide the playback speed:

| Waypoints | Time used |
|---|---|
| State waypoints with times (time-parameterized) | their own times |
| State waypoints all at `t = 0` (not time-parameterized) | their index, one waypoint per second |
| Joint waypoints (they carry no time) | their index, one waypoint per second |

A trajectory you computed yourself goes through `Trajectory`:

```python
from tesseract_robotics.viewer import Trajectory

viewer.update_trajectory(Trajectory.build(joint_names, positions, times_s))
# or rows of [q_1, ..., q_J, t]
viewer.update_trajectory(Trajectory.from_rows(joint_names, rows))
```

### Playback Controls

The **Playback** folder in the viewer's control panel exists from construction, so it is there for a browser that connects late.

| Control | Effect |
|---|---|
| **Play** | Run or pause. `update_trajectory` turns it on, `update_joint_positions` off |
| **Time [s]** | Dragging pauses playback and scrubs |
| **Speed** | Time multiplier; 0 holds the current time |
| **Loop** | Restart at the end, or stop on the last waypoint |

Re-pushing an environment that still has the trajectory's joints keeps it playing. An environment without them drops the trajectory with a logged warning.

## TCP Path

`plot_trajectory` draws the path the program commands, chosen per move by the type of the instruction arriving at each waypoint. MoveL and MoveJ have different true paths, so neither a fitted curve nor straight lines everywhere can draw both. It is the commanded path, not a prediction of the machine's: a controller's blending zones are not in the program and do not appear.

| Move | Drawn between two waypoints |
|---|---|
| `LINEAR` (MoveL) | The straight line between the two TCP positions |
| `FREESPACE` (MoveJ) | FK along the synchronized joint interpolation `q(s) = q_a + s (q_b - q_a)`, uniform in `s`, without angle wrapping |
| `CIRCULAR` | Raises `UnsupportedMoveTypeError` |
| No program (`Trajectory.build`, `from_rows`) | As `FREESPACE`: joint values alone say nothing more |

Axes triads sit on the waypoints only, so a MoveJ sample never looks like a planned pose. The TCP is `tcp_frame` with `tcp_offset` applied, since a MoveL's line belongs to the TCP and not the flange.

A MoveJ is drawn in `n = max(ceil(max|dtheta| / 1 deg), ceil(max|dd| / 50 mm))` pieces, over its revolute and prismatic joints. Without revolute motion one piece is exact.

| Measurement | Result |
|---|---|
| Drawn MoveJ against FK sampled 16x finer, 300 random full-range moves (IRB2400, 8-DOF rail gantry) | 0.102 mm worst |
| One straight chord for those IRB2400 moves | 589 mm median, 2280 mm worst |
| Move types through planning, 11 examples (TrajOpt, OMPL, Descartes, pipelines) | Kept |
| TCP at MoveL waypoints leaving the run's line: `basic_cartesian`, `raster`, `pick_and_place` | 0.000 mm |
| Same for `glass_upright`, whose MoveL targets constrain orientation | 88 mm: the plan's waypoints are off the line, and the plot shows it |

!!! warning "Playback does not follow a MoveL's line"
    The player interpolates joints on every move, so between MoveL waypoints the animated TCP leaves the plotted line: 4.1 mm at most on `basic_cartesian`'s 125 mm moves, 1.6 mm on `raster`'s 100 mm. The waypoints themselves coincide.

!!! note "Moving working frames"
    A MoveL is straight in its `working_frame`. If the group moves that frame (a part on a positioner), its world path is not a line, and `plot_trajectory` raises `MovingWorkingFrameError` rather than draw one. MoveJ draws normally there.

## Markers and Custom Scene Content

The viewer draws links, visuals and `plot_trajectory`. Anything else goes straight to viser through `viewer.server`. See the [viser documentation](https://viser.studio):

```python
viewer.server.scene.add_icosphere(
    "/markers/target", radius=0.02, color=(255, 0, 0), position=(0.8, 0.0, 0.8)
)
viewer.server.gui.add_button("Replan")
```

Keep markers out of `/links/`, which `update_environment` replaces.

## Shading

| Geometry | Shading |
|---|---|
| Mesh whose file has smooth normals | The file's normals, corrected for a non-uniform URDF scale |
| Mesh with facet-only or no normals (STL) | Auto-smoothed: neighbouring faces under 30 degrees share a normal, sharper edges stay creased |
| Textured mesh | The file's normals; never auto-smoothed, since merging vertices would orphan its UVs |
| Sphere, cylinder, capsule, cone | Auto-smoothed, so caps keep their edge |
| Box, octree voxels | Flat |

All materials are double-sided: sheet-metal meshes often mix face windings, which would otherwise show as holes. Colour comes from the URDF `<material>` on the visual, then the mesh's own material or texture, then tesseract's default grey.

## Industrial Workcells

--8<-- "docs/demos/players/twc_workcell_positioner_viewer.html"

The Tesseract Robotics Workcell Collection (TWC) ships rail and positioner cells. Trajectories over any subset of joints, in any order, pose by their own joint names:

```python
--8<-- "src/tesseract_robotics/examples/twc_workcell_positioner_viewer.py:load_workcell"
```

!!! note "Submodule Required"
    TWC workcell assets ship as a git submodule. Initialize with:
    ```bash
    git submodule update --init
    ```

## API Quick Reference

| Member | Purpose |
|---|---|
| `TesseractViewer(host="127.0.0.1", port=8000)` | Start serving; also a context manager that calls `close()` |
| `url` | Address to open in a browser |
| `server` | The `viser.ViserServer`, for markers and GUI |
| `update_environment(env)` | Replace the scene with `env` at its current state |
| `update_joint_positions(joint_names, q)` | Show a stationary pose; stops playback |
| `update_trajectory(trajectory)` | Animate a `CompositeInstruction` or `Trajectory` from its start |
| `show_time(t_s)` | Pose the scene at one trajectory time |
| `plot_trajectory(trajectory, manip_info, axes_length=0.1, color=(255, 255, 255))` | Draw the commanded [TCP path](#tcp-path) with an axes triad per waypoint |
| `close()` | Stop playback and serving, release the port; idempotent |
| `serve_forever()` | Block until Ctrl-C, then `close()` |

Failures raise named errors from `tesseract_robotics.viewer.errors`:

| Error | When |
|---|---|
| `ViewerPortInUseError` | The requested port is taken |
| `NoEnvironmentError` | `update_joint_positions`, `update_trajectory` or `plot_trajectory` before `update_environment` |
| `NoTrajectoryError` | `show_time` before `update_trajectory` |
| `UnknownJointError` | A joint name the environment lacks |
| `StaleSceneError` | The environment gained or lost links since `update_environment` |
| `EmptyTrajectoryError` | A trajectory without waypoints |
| `TrajectoryShapeError` | Names, positions, times and move types disagree, times decrease, or a program's waypoints name different joints |
| `UnplannedWaypointError` | A program still has a Cartesian waypoint |
| `UnsupportedGeometryError` | A visual with no mesh form (`Plane`, ...) |
| `UnsupportedMoveTypeError` | `plot_trajectory` with a move that is neither `LINEAR` nor `FREESPACE` |
| `MovingWorkingFrameError` | `plot_trajectory` with a `LINEAR` move in a working frame the group moves |
| `NamedTcpOffsetError` | `plot_trajectory` with a `tcp_offset` that names a link |
| `ViewerCloseTimeoutError` | The playback thread did not stop in `close()` |

A failure during playback has no caller to raise to: it logs the traceback and pauses. Play or `update_trajectory` try again.

## Examples

- [`shapes_viewer.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/shapes_viewer.py) — primitives
- [`tesseract_material_mesh_viewer.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/tesseract_material_mesh_viewer.py) — materials + meshes
- [`abb_irb2400_viewer.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/abb_irb2400_viewer.py) — robot + planned trajectory
- [`twc_workcell_positioner_viewer.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/twc_workcell_positioner_viewer.py) — industrial workcell (requires `git submodule update --init`)
