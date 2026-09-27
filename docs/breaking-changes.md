# Breaking API changes

Changes that break existing Python code, newest first, each with what replaces it. The next
release removes `IfoptQPProblem` and `IfoptProblem`: build a `TrajOptQPProblem` instead, and
expect different results wherever a cost went in without a penalty type. The
[changelog](CHANGELOG.md) lists every change; the [upgrade guide](changes.md) covers the tesseract
upgrades in full.

| Release | What breaks | Use instead |
|---|---|---|
| Unreleased | `trajopt_sqp.IfoptQPProblem` and `trajopt_sqp.IfoptProblem` removed | [`TrajOptQPProblem`](#ifoptqpproblem-and-ifoptproblem-removed) |
| 0.35.0.1 | `planning.Transform` removed; `Pose` is an `Isometry3d` | [`Pose`](#transform-removed-pose-is-an-isometry3d) |
| 0.35.0.1 | tesseract 0.35: `package://tesseract_support/` resource URIs | [`package://tesseract/support/`](#resource-uris-moved) |
| 0.34.1.0 | tesseract 0.34: `ifopt` module, `JointPosition`, `CartPosInfo`, `CollisionCache`, … | [0.33 → 0.34 guide](changes.md#breaking-changes) |

## Unreleased

### `IfoptQPProblem` and `IfoptProblem` removed

`tesseract_robotics.trajopt_sqp` no longer binds `IfoptQPProblem` or `IfoptProblem`. trajopt is
removing `IfoptQPProblem` (tesseract-robotics/trajopt#595), and `IfoptProblem` was bound only to
construct it. Code that uses either now fails with
`AttributeError: module 'tesseract_robotics.trajopt_sqp' has no attribute 'IfoptQPProblem'`
(or `'IfoptProblem'`). Build a `TrajOptQPProblem` over the same variables, and add every set to
it, each cost with a penalty type (#148):

```python
# before
nlp = tsqp.IfoptProblem(nodes_variables)
nlp.addConstraintSet(joint_constraint)
nlp.addCostSet(vel_cost)                        # no penalty type
problem = tsqp.IfoptQPProblem(nlp)
problem.addConstraintSet(collision_constraint)
problem.setup()

# after
problem = tsqp.TrajOptQPProblem(nodes_variables)
problem.addConstraintSet(joint_constraint)
problem.addConstraintSet(collision_constraint)
problem.addCostSet(vel_cost, tsqp.CostPenaltyType.SQUARED)
problem.setup()
```

It is not a rename. In trajopt 0.35.0 the two problems differ in five ways, and the third and
fourth change results:

| | `IfoptQPProblem` (removed) | `TrajOptQPProblem` |
|---|---|---|
| **1. Per set, not per row** | One merit coefficient, name and violation per constraint row; names are `<set>_<row>` ([L93][i93], [L100][i100]). | One per constraint set ([L668][t668], [L675][t675], [L1021–L1043][t1021]). Code that indexes names, violations or merit coefficients by row breaks, and the solver inflates a whole set's merit coefficient at once. |
| **2. `addCostSet` checks bounds** | Checks nothing; rejects `HINGE` ([L50–L78][i50]). | `SQUARED` and `ABSOLUTE` need equality bounds on every row, `HINGE` one-sided bounds; anything else, a range row included, raises ([L416–L476][t416]). |
| **3. No raw-cost path** | A cost added with `IfoptProblem.addCostSet` bypasses `IfoptQPProblem::addCostSet` ([L59][i59]): it gets no gradient and no Hessian in the QP ([L190][i190], [L263][i263]), while the merit still reads its raw, signed rows ([L525–L531][i525]). | Every cost goes in with `addCostSet(set, penalty_type)`. A cost that was inert now shapes the steps. |
| **4. The merit matches the model** | Weights the model but not the merit, and counts the model once per cost term, so the solver can reject every step and stop at the seed (tesseract-robotics/trajopt#595). | The merit weights and counts costs as the model does, so steps the old ratio rejected can be accepted. Constraint coefficients are the exception on 0.35.0: they weight a set's slack in the QP ([L798][t798]) but not its violation in the merit ([L1021–L1043][t1021]); tesseract-robotics/trajopt#592 fixes that. |
| **5. Dynamic sets** | Throws on `isDynamic()` sets ([L42–L48][i42]). | Accepts them ([L404–L414][t404]). |

How much the third difference moves a result: this package's online SQP example added its velocity
cost with `IfoptProblem.addCostSet`, so its QP had no cost term and OSQP solved with an empty
Hessian. As a squared cost on `TrajOptQPProblem`, the global solve's summed squared joint steps
fell from 23.2 to 3.9, against 3.6 for the straight-line seed, and `getTotalExactCost()` now reports
that sum instead of the first velocity row (0.5).

!!! note "Evidence"
    The five differences are read from the trajopt 0.35.0 sources linked above. The example's
    numbers are measured on this repository's `online_planning_sqp_example.py`, and
    `tests/examples/test_online_planning_sqp_example.py` pins its problem class, its cost as the
    squared joint steps, and a global solve that reaches the target from the pinned start. How
    much results move in other code depends on its costs; nothing here measures that.

[i42]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L42-L48
[i50]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L50-L78
[i59]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L59
[i93]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L93
[i100]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L100
[i190]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L190
[i263]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L263
[i525]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/ifopt_qp_problem.cpp#L525-L531
[t404]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L404-L414
[t416]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L416-L476
[t668]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L668
[t675]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L675
[t798]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L798
[t1021]: https://github.com/tesseract-robotics/trajopt/blob/0.35.0/trajopt_optimizers/trajopt_sqp/src/trajopt_qp_problem.cpp#L1021-L1043

## 0.35.0.1

### `Transform` removed; `Pose` is an `Isometry3d`

`tesseract_robotics.planning.Pose` now subclasses `Isometry3d`. Its rotation math runs in Eigen,
its numpy-property facade is gone, its stored data are properties, and its factories collapse to
one name each that accepts both scalar-positional and array-like arguments. The `Transform` alias
is removed: use `Pose` (#76).

### Resource URIs moved

tesseract 0.35 folds its support data into the `tesseract` package, so
`package://tesseract_support/...` is now `package://tesseract/support/...` in every URDF, SRDF and
`locateResource` call. The rest of that upgrade leaves the Python API unchanged; see the
[0.34 → 0.35 guide](changes.md#urdfsrdf-resource-uri-scheme).

## 0.34.1.0

The tesseract 0.34 upgrade removed the `ifopt` module (its types moved to `trajopt_ifopt`) and
replaced `JointPosition` with `Var` / `Node` / `NodesVariables`. It also removed `CartPosInfo` and
`CollisionCache`, dropped `getStaticKey()` from profiles, and renamed the cost and constraint
evaluation methods. The [0.33 → 0.34 guide](changes.md#breaking-changes) has each change with its
replacement.
