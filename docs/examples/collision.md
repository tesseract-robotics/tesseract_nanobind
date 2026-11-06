# Collision Examples

Collision checking below the high-level `Robot` API: contact-manager plugins,
collision margins, contact validation, continuous checks and convex
decomposition. Each example is a
port of an upstream tesseract example or test, or takes its parameters from
upstream source, cited in the module docstring. Every value printed is also
asserted in `tests/examples`.

```bash
# Installed console scripts
tesseract_contact_managers_example
tesseract_trajectory_collision_example
tesseract_convex_decomposition_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.contact_managers_example
```

## Contact Managers

A port of upstream's `box_box_example.cpp`, run on every discrete backend, then
the rest of the contact-manager API on the same scene.

Load the plugin factory from `contact_manager_plugins.yaml`. It names three
discrete managers (`BulletDiscreteBVHManager` is the default) and two
continuous ones (`BulletCastBVHManager` is the default):

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:factory"
```

The second box is upstream's `box_2m.ply` as a convex hull. Upstream reads the
PLY with `loadSimplePlyFile`, which is not bound; `createMeshFromPath` loads the
same mesh, and `makeConvexMesh` is the call upstream makes on it:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:hull"
```

The example itself: a unit box inside the 2 m hull, then moved to x = 1.6, a
0.1 m gap, and checked at margins 0.1 and 0.25.

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:box_box"
```

| backend | inside: pair | inside: distance | x = 1.6, margin 0.1 | x = 1.6, margin 0.25 |
|---|---|---|---|---|
| `BulletDiscreteBVHManager` | box_link / second_box_link | −1.2663 | no contact | 0.1000 |
| `BulletDiscreteSimpleManager` | box_link / second_box_link | −1.2663 | no contact | 0.1000 |
| `FCLDiscreteBVHManager` | box_link / second_box_link | −1.3000 | no contact | 0.1000 |

The backends agree on which pair collides, not on how deep: the analytic
penetration is 1.3 m, which FCL reports. Upstream's own box-box unit test
avoids `box_2m.ply` for Bullet for this reason (`collision_box_box_unit.hpp:58`).
`BulletDiscreteSimpleManager` also reports `link_names` in the other order, so
the example compares the `ContactResultMap` keys, which are always ordered.

!!! note "The margin 0.1 case is a boundary"
    Both backends keep a contact whose distance is *at most* the margin
    (`bullet_utils.cpp:1072`, `fcl_utils.cpp:307`). In doubles the gap is
    `(1.6 - 0.5) - 1.0 = 0.10000000000000009`, one ulp above 0.1, so margin
    0.1 reports nothing, as upstream's unit test expects
    (`collision_box_box_unit.hpp:201`). Any margin even 1e-12 m larger reports
    the pair.

A manager can also be created from a `PluginInfo`:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:plugin_info"
```

### Collision margins

The pair-margin cases of upstream's box-box unit test, built with the
pair-data constructors and checked by a contact test at the 0.1 m gap pose. A
margin on a pair that isn't in the scene (1.7 m) raises
`getMaxCollisionMargin()` but not `getMaxCollisionMargin("box_link")`, so the
boxes stay out of contact. `incrementMargins(0.15)` and `scaleMargins(2.5)`
take 0.1 to upstream's second margin, 0.25. `apply` with `MODIFY` merges pair
margins and with `REPLACE` swaps them:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:margins"
```

| case | max | max for `box_link` | contact |
|---|---|---|---|
| default 0.1 + unrelated pair 1.7 | 1.7 | 0.1 | no |
| default 0.1 + box pair 0.25 | 0.25 | 0.25 | yes |
| box pair 0.25 only | 0.25 | 0.25 | yes |
| 0.1, `incrementMargins(0.15)` | 0.25 | 0.25 | yes |
| 0.1, `scaleMargins(2.5)` | 0.25 | 0.25 | yes |
| unrelated 1.7, `apply(box pair 0.25, MODIFY)` | 1.7 | 0.25 | yes |
| unrelated 1.7, `apply(box pair 0.25, REPLACE)` | 0.25 | 0.25 | yes |

`CollisionMarginPairData` has the same arithmetic. Its
`getMaxCollisionMargin(link)` returns `None` for a link no pair involves:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:pair_data"
```

### Editing the factory

Add a copy of a plugin, make it the default, create a manager from it and
remove it again, as upstream's `PluginFactorAPIUnit` does. A manager created by
plugin name carries that name. When the default is removed, the first remaining
plugin by name becomes the default:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:edit_plugins"
```

`saveConfig` writes the factory's current configuration, and a factory built
from that file reports the same `getConfig()`:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:save_config"
```

`clearSearchPaths` and `clearSearchLibraries` empty the plugin loader's
search lists:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:clear_search"
```

### Allowed collision matrix

Upstream's `TestGetAllowedCollisions` entries, built from a dict keyed by
`makeOrderedLinkPair`. The result equals an ACM built entry by entry with
`addAllowedCollision`. `getAllowedCollisions` lists the links allowed to collide
with any of `link1`, `link3`, `cause_duplicate`: 5 names, or 6 when duplicates
are kept. `removeAllowedCollision("link1")` drops every entry involving `link1`:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:acm"
```

### Contact validation and result maps

A `ContactResultValidator` subclass approves or rejects each contact. A contact
test calls it from its worker thread with the GIL taken again, and an exception
it raises propagates to the caller:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:validator_class"
```

With the thin box enabled, the three objects overlap pairwise. The validator is
called once directly, then set on `ContactRequest.is_valid`, where it drops the
`box_link` / `thin_box_link` pair:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:validator"
```

A `ContactResultMap` iterates as `(key, ContactResultVector)` pairs. `filter`
edits the vectors in place, and `addContactResult` / `setContactResult` take
either one result or a vector:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:result_map"
```

### Continuous (cast) manager

Upstream's box-box cast unit test: a 0.25 m box swept from (−1.9, 0, 0) to
(1.9, 3.8, 0) past a static unit box. Each of the three cast overloads of
`setCollisionObjectsTransform` sets the same sweep, by name, by name list or by
a pair of transform maps:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:cast"
```

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:cast_names"
```

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:cast_maps"
```

Each gives upstream's values: distance −0.2475, `cc_time` (−1, 0.25), and the
contact point mapped through the sweep's start and end poses,
`transform[1] * nearest_points_local[1]` = (−1.275, −0.625, 0) and
`cc_transform[1] * nearest_points_local[1]` = (2.525, 3.175, 0).

!!! note "Parking a cast object"
    A single-pose `setCollisionObjectsTransform` on a cast manager moves the
    object but keeps its previous sweep (`bullet_cast_bvh_manager.cpp:240–256`).
    To park an object, set a zero-length sweep: the same pose twice.

### Environment contact managers

On the IIWA environment, `EnvironmentContactAllowedValidator` answers from the
scene graph's ACM: `base_link` / `link_1` is disabled in the SRDF ("Adjacent"),
`base_link` / `link_5` is not. Any configured manager can be fetched by name,
and the `SetActive*ContactManagerCommand` pair switches the active ones:

```python title="contact_managers_example.py"
--8<-- "src/tesseract_robotics/examples/contact_managers_example.py:environment"
```

## Trajectory Collision

Collision checks of a single state, a swept segment and a whole trajectory, and
a trajectory turned into a collision link.

### States and segments

Upstream's boxbot test: two unit boxes, `test_box_link` 0.9 m along x, so they
overlap by 0.1 m. `checkTrajectoryState` has a discrete and a continuous
overload; both report the pair:

```python title="trajectory_collision_example.py"
--8<-- "src/tesseract_robotics/examples/trajectory_collision_example.py:state"
```

`checkTrajectorySegment` sweeps `test_box_link` from y = 2 to y = −2 through
`boxbot_link`:

```python title="trajectory_collision_example.py"
--8<-- "src/tesseract_robotics/examples/trajectory_collision_example.py:segment"
```

The penetration depth depends on the backend. FCL, fetched by name from the
boxbot's plugin configuration, reports the analytic 0.1 m:

```python title="trajectory_collision_example.py"
--8<-- "src/tesseract_robotics/examples/trajectory_collision_example.py:state_fcl"
```

| manager | distance |
|---|---|
| `BulletDiscreteBVHManager` (default discrete) | −0.0577 |
| `BulletCastBVHManager` (default continuous) | −0.0577 |
| `FCLDiscreteBVHManager` | −0.1000 |

Bullet's −0.0577 m is 0.1 / √3, along a corner direction. The boxes' y and z
faces are coplanar, and Bullet's penetration solver does not find the x-axis
minimum. Upstream's boxbot test asserts only that the pair is in contact.
Disabling `boxbot_link` through `ContactManagerConfig.modify_object_enabled`
clears every check.

### `checkTrajectory`, rebuilt from its parts

Upstream's `checkTrajectoryUnit` (IIWA with a 0.15 m sphere attached to
`base_link`, five states from `joint_a1` = −0.4 to 0.4 rad) checked with
`LVS_CONTINUOUS`. `checkTrajectory` splits each step into sub-segments of at
most `longest_valid_segment_length` (0.005, the default) in joint space,
checks each with `checkTrajectorySegment` and merges the results with
`addInterpolatedCollisionResults`. The example does the same in Python:

```python title="trajectory_collision_example.py"
--8<-- "src/tesseract_robotics/examples/trajectory_collision_example.py:lvs_rebuild"
```

Each step's rebuilt result equals `checkTrajectory`'s, and both give
upstream's counts: link pairs per step 2, 2, 3, 2, and 45 + 80 + 118 + 45 = 288
contacts, with the most in step 2.

!!! note "`ContactResultMap.clear()` keeps its keys"
    `clear()` empties every vector but keeps the key, and `==` compares keys.
    `checkTrajectory` reuses one map across steps (`utils.cpp:234`), so in step
    3 the pair `link_5` / `sphere_attached`, which touched only in step 2, is
    still there with no contacts. A rebuild with a new map per step has the same
    contacts but compares unequal. Reuse one map, as above.

### `JointTrajectory`

Upstream's trajectory-link test trajectory, `joint_a1` and `joint_a2` from 0 to
1 rad, built with either constructor. Indexing returns a copy; assign it back
with `traj[i] = state`:

```python title="trajectory_collision_example.py"
--8<-- "src/tesseract_robotics/examples/trajectory_collision_example.py:joint_trajectory"
```

### Trajectory links

`AddTrajectoryLinkCommand` adds a link, `traj_link` under `base_link`, that
carries the collision geometry of every active link over the trajectory. The
`Method` decides how that geometry is merged. The example applies each method
in turn, replacing the link (`replace_allowed=True` after the first):

```python title="trajectory_collision_example.py"
--8<-- "src/tesseract_robotics/examples/trajectory_collision_example.py:trajectory_link"
```

`joint_a1` and `joint_a2` move `link_1` to `link_7`, one collision shape each,
over 2 states (`environment.cpp:1412–1471`):

| `Method` | collision shapes | rule |
|---|---|---|
| `PER_STATE_OBJECTS` | 14 | every active link's shapes, per state (7 × 2) |
| `PER_STATE_CONVEX_HULL` | 2 | one hull per state |
| `GLOBAL_PER_LINK_CONVEX_HULL` | 7 | one hull per active link |
| `GLOBAL_CONVEX_HULL` | 1 | one hull for everything |

## Convex Decomposition

A port of upstream's `create_convex_decomposition` command-line tool: split a
mesh into convex hulls with V-HACD and write each hull to a PLY file. The input
is the car-seat visual mesh `seat.dae` (10367 vertices, 20776 faces).

### Options

Upstream's options, with its short flags and value ranges, as `argparse`. The
defaults are `VHACDParameters()`'s, and every field is set from an option:

```python title="convex_decomposition_example.py"
--8<-- "src/tesseract_robotics/examples/convex_decomposition_example.py:options"
```

The example deviates from the tool in three places: `--input` and `--output`
default to `seat.dae` and `seat.ply`, so it runs without arguments; any format
Assimp reads is accepted, where upstream reads only PLY (`loadSimplePlyFile`,
not bound); and `--fill_mode` is added, because the fill mode changes the
result. As upstream, a range is checked only for a value given on the command
line: the default `max_convex_hulls` of 64 lies outside `-n`'s range [1, 32].

```bash
tesseract_convex_decomposition_example -n 16 --fill_mode RAYCAST_FILL -o /path/to/seat.ply
```

### Decompose and write

```python title="convex_decomposition_example.py"
--8<-- "src/tesseract_robotics/examples/convex_decomposition_example.py:decompose"
```

At the defaults this gives 64 hulls in about a second. Each is written as
`<i>_<output>`, as the tool does:

```python title="convex_decomposition_example.py"
--8<-- "src/tesseract_robotics/examples/convex_decomposition_example.py:write"
```

Read back, each file holds its hull's vertices to the PLY's 7 written decimals.

Each hull is checked for convexity against its own faces: every face is planar
and every vertex lies on the inner side of every face plane, within the
rounding of Bullet's `int * scaling + center` output and of the plane test
(Shewchuk's `orient3d`). V-HACD can also return a flat hull, all its vertices in
one plane, with a front and a back face. That hull is convex when each face is a
convex polygon in its plane (`orient2d`). On macOS arm64 the seat's default
decomposition has no flat hull; `box_2m.ply` at resolution 10000 with
`SURFACE_ONLY` has 28 of 64.
A second `createConvexHull` of its vertices passes the same test and keeps only
the hull's own vertices, up to one step of Bullet's integer grid.

!!! note "`createConvexHull` is not idempotent"
    Bullet computes a hull on a grid of (max − min) / 10216 per axis
    (`btConvexHullComputer.cpp:1992`). A vertex within one grid step of a face
    is merged away, so hulling a hull's vertices again can drop one: one
    `RAYCAST_FILL` hull of the seat loses a vertex 4 µm above a face. Compare
    vertex sets within the grid step, not vertex counts.

### Fill modes

`FillMode` decides which voxels count as inside the mesh: `FLOOD_FILL` for
closed meshes, `RAYCAST_FILL` for open ones, `SURFACE_ONLY` for hollow ones.
Each gives its own decomposition. The default-constructed
`ConvexDecompositionVHACD()` gives the same hulls as explicit default parameters:

```python title="convex_decomposition_example.py"
--8<-- "src/tesseract_robotics/examples/convex_decomposition_example.py:fill_modes"
```

| `FillMode` | hulls | bounding box (m) |
|---|---|---|
| `FLOOD_FILL` | 64 | (−0.3641, −0.2366, 0.0) to (0.2309, 0.2396, 1.0035) |
| `SURFACE_ONLY` | 64 | (−0.3668, −0.2366, 0.002) to (0.2309, 0.2396, 1.0049) |
| `RAYCAST_FILL` | 64 | (−0.3668, −0.2366, 0.0) to (0.2306, 0.2395, 1.0049) |
| mesh `seat.dae` | | (−0.3669, −0.2403, 0.0) to (0.2365, 0.2403, 1.0094) |
| shipped `seat_*.stl` | 10 | (−0.367, −0.2404, 0.0) to (0.2367, 0.2404, 1.0096) |

The decomposition shipped with tesseract support, `car_seat/collision/seat_*.stl`,
was made with other, unrecorded parameters. It is reported, not compared.

## Signed Distance Field

--8<-- "docs/demos/players/sdf_collision_example.html"

A concave obstacle as a `tesseract_geometry.SignedDistanceField`: a ball with a bore through it,
which no convex hull can represent. A probe sweeps into it and back, so its contact distances
cross the margin band. The voxel shell shows the field's zero level set; the contact manager
queries the field itself.

```bash
tesseract_sdf_collision_example
```
