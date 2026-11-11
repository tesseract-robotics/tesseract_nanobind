# Planning examples

Motion planning examples from simple freespace to complex industrial tasks.

The OMPL examples (freespace, hybrid OMPL + TrajOpt, objectives from Python) have their
own page: [OMPL Examples](ompl.md).

All examples install as console scripts. Run them with:

```bash
# Installed console script (preferred)
tesseract_basic_cartesian_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.basic_cartesian_example
```

## Basic Cartesian

--8<-- "docs/demos/players/basic_cartesian_example.html"

Cartesian straight-line motion with TrajOpt, past an obstacle built from point-cloud data: the
C++ example's octomap, a 1 m cube of points voxelised into 1000 boxes of 0.1 m — the collision
load a scanned part presents. Replace the generated grid with a scanner's points and the rest of
the example is unchanged:

```bash
tesseract_basic_cartesian_example
```

See the source at
[`basic_cartesian_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/basic_cartesian_example.py).

## Glass upright (orientation constraint)

--8<-- "docs/demos/players/glass_upright_example.html"

Keep end-effector orientation constrained (e.g. carrying a glass of water) —
demonstrates 6-DOF Cartesian constraints inside a TrajOpt program:

```bash
tesseract_glass_upright_example
```

See the source at
[`glass_upright_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/glass_upright_example.py).

## Pick and place

--8<-- "docs/demos/players/pick_and_place_example.html"

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

## Raster pattern

--8<-- "docs/demos/players/raster_example.html"

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

## Multi-step with task composer (car seat)

--8<-- "docs/demos/players/car_seat_example.html"

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

## Descartes + TrajOpt chaining

--8<-- "docs/demos/players/chain_example.html"

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

## 9-DOF with external axis (puzzle piece)

--8<-- "docs/demos/players/puzzle_piece_auxillary_axes_example.html"

A Cartesian toolpath along a part's edge, planned for nine joints: a 7-DOF KUKA IIWA and a
2-DOF positioner (turntable and tilt) that holds the part. The toolpath is given in the part's
frame, which moves with the positioner, and TrajOpt distributes the motion over all nine joints. The plan takes
33 to 34 s on an Apple M1 Max (10 cores).

```bash
tesseract_puzzle_piece_auxillary_axes_example
```

See the source at
[`puzzle_piece_auxillary_axes_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/puzzle_piece_auxillary_axes_example.py).

## Visualization

--8<-- "docs/demos/players/abb_irb2400_viewer.html"

All examples can be visualized:

<!-- snippet: skip (starts the viewer on its default port and waits for Enter) -->
```python
from tesseract_robotics.viewer import TesseractViewer

with TesseractViewer() as viewer:
    viewer.update_environment(robot.env)
    if result.successful:
        viewer.update_trajectory(result.raw_results)
    print(f"Viewer at {viewer.url}")
    input("Press Enter to exit...")
```

## Next steps

- [OMPL Examples](ompl.md) - Sampling-based planning and optimization objectives
- [Online Planning](online-planning.md) - Replanning examples
- [Low-Level SQP Guide](../user-guide/low-level-sqp.md) - Direct solver access
