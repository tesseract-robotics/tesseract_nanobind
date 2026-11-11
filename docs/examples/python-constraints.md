# Python Constraints

Seven constraints written in Python, switched on one after another in a live loop.

--8<-- "docs/demos/players/python_constraints_example.html"

An ABB IRB 2400 rides a rail, with a grinder on its flange. The tool tip rests on a mark on a
workpiece. One TrajOpt problem is solved per frame: a single waypoint, warm-started from the
frame before, with a squared stay-close cost and whichever rules of
[`trajopt_ifopt.python_constraints`](../user-guide/low-level-sqp.md#pyconstraint-a-rule-from-fq)
are on. Each rule is a few lines of Python; the SQP that satisfies them is TrajOpt's, in C++.

## What you are watching

| Act | Rules on | What happens |
|---|---|---|
| 1 | none | The rail moves and the tool goes with it. |
| 2 | `WorldHold` | The tip stays on the mark while the rail moves. The tool axis wanders. |
| 3 | + `HoldAxis` 0.5° | The tool axis locks onto vertical. |
| 4 | + `WatchTheTool` 3° | A camera on a mast on the carriage turns to keep the tip in its cone. It stays on from here. |
| 5 | `PointsDown` 20° + `ElbowUp` | The rail becomes the solver's seventh axis. A plane rises under the elbow, and the carriage drives to where the elbow clears it. |
| 6 | + `StayDexterous` | Manipulability stays above a rising bound. |
| 7 | + `Clearance` | A ball drives in. The forearm keeps 15 cm from it, and the tip stays put. |
| 8 | all seven | The mark moves, and the tip traces a circle on the workpiece. |

In acts 1 to 4 something else moves the rail: its position is measured, and the arm and the
camera solve in the carriage's frame. From act 5 the rail is a variable like any joint.

Every bound that moves ramps from the live value, so nothing jumps when a rule comes on.

## Reading the scene

- **The panel**, on the plane behind the robot, has a ruler per rule. The ball on a ruler is
  the live value. The bar is the bound, and it is there only while the rule is on.
- **The last ruler is the solve.** Its ball is what the frame's solve took, and its bar is
  the frame the loop has at 30 Hz, 33 ms.
- **The glyphs** show each rule in the cell: the mark, the `HoldAxis` line, the `PointsDown`
  cone, the `ElbowUp` plane, the camera's cone and optical axis, and the shell around the ball
  that the forearm stays out of.
- **The caption** names the act, and says how many rules are on and what a frame's solve
  took, as the act's median.

The captions, the gauges' markers, the mark and the glyphs are links of the environment on
prismatic joints, posed by the same trajectory as the robot. That is how a recording, which can animate
link poses and nothing else, shows a bound appear, move and disappear.

## The rules of one frame

```python title="python_constraints_example.py (rules)"
--8<-- "src/tesseract_robotics/examples/python_constraints_example.py:rules"
```

## The loop

`LiveLoop.step` solves one frame. `run` plays the acts through it.

```python title="python_constraints_example.py (loop)"
--8<-- "src/tesseract_robotics/examples/python_constraints_example.py:loop"
```

## The acts

```python title="python_constraints_example.py (acts)"
--8<-- "src/tesseract_robotics/examples/python_constraints_example.py:acts"
```

## The cell

The tool, the rail, the workpiece, the camera and the ball are added to tesseract's
`abb_irb2400_on_positioner` scene. Their meshes carry a `MeshMaterial` with metallic and
roughness values, which the viewer shades with.

```python title="python_constraints_example.py (cell)"
--8<-- "src/tesseract_robotics/examples/python_constraints_example.py:cell"
```

## Drive it yourself

```bash
tesseract_python_constraints_example            # the acts play by themselves
tesseract_python_constraints_example --sliders  # you drive
```

Without the flag the acts play, and the viewer's Playback controls start, stop and scrub them.

With `--sliders` the loop solves from the viewer's control panel instead:

- **Rail swings by itself** starts and stops the rail's own motion; unticked, the **Rail**
  slider is yours.
- **Rail is the solver's 7th axis** hands the carriage to the solver. It drives to abeam of
  the tip and stays within the band of **Carriage follows the tip within**. Widen the band,
  and the other rules move the carriage instead.
- **The mark has a handle.** Drag it: along the rail the carriage follows, and across it the
  arm reaches.
- **Ball on its slide** drives the ball into the arm.
- Every rule has a checkbox, and a slider where it has a bound. A bound that comes on closes
  in from the live value, as the acts' bounds ramp.
- **Solve** shows the time of the last frame, and says when the rules you ticked conflict.

Either way, **Constraint opacity** sets how solid the rules' glyphs are drawn.

```python title="python_constraints_example.py (sliders)"
--8<-- "src/tesseract_robotics/examples/python_constraints_example.py:sliders"
```

## A rule of your own

The carriage follows the tip because of an eighth rule, written in the example. With the rail
handed over and no rule asking, the solver leaves the carriage where it is: the only cost is
staying close to the frame before. `CarriageFollows` asks.

```python title="python_constraints_example.py (own_rule)"
--8<-- "src/tesseract_robotics/examples/python_constraints_example.py:own_rule"
```

## Solve times

Per frame, on a shared 2-vCPU cloud VM (Intel Xeon, 2.1 GHz). Indicative: run the example for
local numbers, which it prints per act.

| Act | Rules | Median | Slowest frame |
|---|---|---|---|
| 2 | 1 | 0.9 ms | 1.4 ms |
| 3 | 2 | 2.4 ms | 5.1 ms |
| 4 | 3 | 3.9 ms | 7.0 ms |
| 5 | 4 | 4.6 ms | 8.6 ms |
| 6 | 4 | 4.6 ms | 16.0 ms |
| 7 | 4 | 3.9 ms | 24.0 ms |
| 8 | 7 | 11.3 ms | 30.0 ms |

`StayDexterous`, `WatchTheTool`, `ElbowUp` and `Clearance` use the default forward-difference
Jacobian; `Clearance` queries the contact manager for each column.

## See also

- [`PyConstraint`: a rule from `f(q)`](../user-guide/low-level-sqp.md#pyconstraint-a-rule-from-fq)
- [Online Planning](online-planning.md) — replanning a whole trajectory per step
- [`python_constraints_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/python_constraints_example.py)
