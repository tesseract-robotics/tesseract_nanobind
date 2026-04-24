# Planning Examples

Motion planning examples from simple freespace to complex industrial tasks.

All examples install as console scripts. Run them with:

```bash
# Installed console script (preferred)
tesseract_freespace_ompl_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.freespace_ompl_example
```

## Freespace OMPL

Basic OMPL planning between joint configurations using the ABB IRB2400:

```bash
tesseract_freespace_ompl_example
```

See the source at
[`freespace_ompl_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/freespace_ompl_example.py).

## Basic Cartesian

Cartesian straight-line motion with TrajOpt:

```bash
tesseract_basic_cartesian_example
```

See the source at
[`basic_cartesian_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/basic_cartesian_example.py).

## Glass Upright (Orientation Constraint)

Keep end-effector orientation constrained (e.g. carrying a glass of water) —
demonstrates 6-DOF Cartesian constraints inside a TrajOpt program:

```bash
tesseract_glass_upright_example
```

See the source at
[`glass_upright_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/glass_upright_example.py).

## Pick and Place

Complete pick and place workflow with object attachment. A KUKA IIWA picks a
box from a table, the box is reparented to `iiwa_tool0` via
[`move_link()`](../user-guide/environment.md), then it is placed on a shelf.

```python title="pick_and_place_example.py"
--8<-- "src/tesseract_robotics/examples/pick_and_place_example.py:full_workflow"
```

Run it:

```bash
tesseract_pick_and_place_example
```

Full source:
[`pick_and_place_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/pick_and_place_example.py).

## Raster Pattern

Industrial raster pattern for welding / painting / milling. Multiple linear
passes with freespace transitions between segments. The example builds the
program programmatically from a list of Cartesian waypoints, then hands it to
the `TrajOptPipeline`.

**Building the program:**

```python title="raster_example.py (build_program)"
--8<-- "src/tesseract_robotics/examples/raster_example.py:build_program"
```

**Planning it:**

```python title="raster_example.py (plan)"
--8<-- "src/tesseract_robotics/examples/raster_example.py:plan"
```

Run it:

```bash
tesseract_raster_example
```

Full source:
[`raster_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/raster_example.py).

## Hybrid Planning (OMPL + TrajOpt)

OMPL finds a feasible path, TrajOpt smooths and optimizes it:

```bash
tesseract_freespace_hybrid_example
```

See the source at
[`freespace_hybrid_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/freespace_hybrid_example.py).

## Multi-Step with Task Composer (Car Seat)

Complex multi-phase sequence: an 8-DOF system (linear carriage + 7-axis arm)
picks a car seat, attaches it via scene graph manipulation, then places it
in a vehicle body. The snippet below is the core sequential-planning loop.

```python title="car_seat_example.py (multi_step)"
--8<-- "src/tesseract_robotics/examples/car_seat_example.py:multi_step"
```

Run it:

```bash
tesseract_car_seat_example
```

Full source:
[`car_seat_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/car_seat_example.py).

## Descartes + TrajOpt Chaining

The `CartesianPipeline` runs Descartes (sampling-based, ladder-graph IK
selection) followed by TrajOpt (smoothing). The snippet below shows the
program + pipeline assembly:

```python title="chain_example.py (descartes_then_trajopt)"
--8<-- "src/tesseract_robotics/examples/chain_example.py:descartes_then_trajopt"
```

Run it:

```bash
tesseract_chain_example
```

Full source:
[`chain_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/chain_example.py).

## 9-DOF with External Axis (Puzzle Piece)

Cartesian toolpath planning with a 9-DOF system (robot + positioner + rail):

```bash
tesseract_puzzle_piece_auxillary_axes_example
```

See the source at
[`puzzle_piece_auxillary_axes_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/puzzle_piece_auxillary_axes_example.py).

## Visualization

All examples can be visualized:

```python
from tesseract_robotics.viewer import TesseractViewer

viewer = TesseractViewer()
viewer.update_environment(robot.env, [0, 0, 0])

# Show trajectory
if result.successful:
    viewer.update_trajectory(result.raw_results)

# Start server
viewer.start_serve_background()
print("Open http://localhost:8000 in browser")
input("Press Enter to exit...")
```

## Next Steps

- [Online Planning](online-planning.md) - Real-time replanning examples
- [Low-Level SQP Guide](../user-guide/low-level-sqp.md) - Direct solver access
