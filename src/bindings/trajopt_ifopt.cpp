/**
 * @file trajopt_ifopt.cpp
 * @brief nanobind bindings for trajopt_ifopt variables, constraints, and costs
 *
 * NOTE: Default arguments for Eigen types (Isometry3d, Matrix) cause std::bad_cast
 * at module load time. All Eigen defaults are avoided - use default constructor
 * and set members instead.
 */

#include "trajopt_ifopt_bindings.h"

NB_MODULE(_trajopt_ifopt, m) {
    m.doc() = "trajopt_ifopt Python bindings - variables, constraints, and costs for IFOPT-based trajectory optimization";

    // JointTrajectory (toJointTrajectory)
    nb::module_::import_("tesseract_robotics.common._common");
    // Import tesseract_collision for CollisionCheckConfig type
    nb::module_::import_("tesseract_robotics.collision");
    // JointGroup / KinematicGroup and Environment: constructor arguments and Info fields
    nb::module_::import_("tesseract_robotics.kinematics");
    nb::module_::import_("tesseract_robotics.environment");

    nb::exception<DegenerateAxisError>(m, "DegenerateAxisError", PyExc_ValueError)
        .attr("__doc__") = "A CartAxis axis argument is zero or not finite.";
    nb::exception<ConstraintNotLinkedError>(m, "ConstraintNotLinkedError", PyExc_RuntimeError)
        .attr("__doc__") = "getJacobian() on a constraint set that is not linked to variables yet.";
    nb::exception<BoundsSizeError>(m, "BoundsSizeError", PyExc_ValueError)
        .attr("__doc__") = "A bounds list whose length is not the number of rows or values it bounds.";
    nb::exception<JointValuesSizeError>(m, "JointValuesSizeError", PyExc_ValueError)
        .attr("__doc__") = "joint_vals whose length is not the constraint's number of joints.";
    nb::exception<VarSizeMismatchError>(m, "VarSizeMismatchError", PyExc_ValueError)
        .attr("__doc__") = "toTrajArray over Vars of different sizes.";
    nb::exception<VarNotInVariableSetError>(m, "VarNotInVariableSetError", PyExc_ValueError)
        .attr("__doc__") = "A constraint linked to a variable set that does not hold its Vars, or a collision "
                           "constraint built on a Var that is not in a NodesVariables.";
    nb::exception<NodeOwnershipError>(m, "NodeOwnershipError", PyExc_ValueError)
        .attr("__doc__") = "A Node handed to NodesVariables that is in a variable set already, or twice.";
    nb::exception<NodeParentUnavailableError>(m, "NodeParentUnavailableError", PyExc_RuntimeError)
        .attr("__doc__") = "Node.getParent on a Node of a variable set C++ owns (a planner's).";
    nb::exception<NumericalJacobianSizeError>(m, "NumericalJacobianSizeError", PyExc_ValueError)
        .attr("__doc__") = "calcForwardNumJac over a function whose output length changes between calls.";

    // ConstraintSet's trampoline is called from trajopt_sqp (TrajOptQPProblem, the solver)
    register_python_error_translator();

    // ========== BoundsType enum ==========
    nb::enum_<ti::BoundsType>(m, "BoundsType")
        .value("UNBOUNDED", ti::BoundsType::kUnbounded)
        .value("EQUALITY", ti::BoundsType::kEquality)
        .value("LOWER_BOUND", ti::BoundsType::kLowerBound)
        .value("UPPER_BOUND", ti::BoundsType::kUpperBound)
        .value("RANGE_BOUND", ti::BoundsType::kRangeBound);

    // ========== RangeBoundHandling enum ==========
    nb::enum_<ti::RangeBoundHandling>(m, "RangeBoundHandling")
        .value("KEEP_AS_IS", ti::RangeBoundHandling::kKeepAsIs)
        .value("SPLIT_TO_TWO_INEQUALITIES", ti::RangeBoundHandling::kSplitToTwoInequalities);

    // ========== Bounds ==========
    nb::class_<ti::Bounds>(m, "Bounds")
        .def(nb::init<double, double>(), "lower"_a = 0.0, "upper"_a = 0.0,
             "Create bounds with lower and upper limits")
        .def("set", &ti::Bounds::set, "lower"_a, "upper"_a,
             "Set both lower and upper bounds")
        .def("setLower", &ti::Bounds::setLower, "lower"_a,
             "Set the lower bound")
        .def("getLower", &ti::Bounds::getLower,
             "Get the lower bound value")
        .def("setUpper", &ti::Bounds::setUpper, "upper"_a,
             "Set the upper bound")
        .def("getUpper", &ti::Bounds::getUpper,
             "Get the upper bound value")
        .def("getType", &ti::Bounds::getType,
             "Get the cached bound classification")
        // C++ declares both operators returning void; nb::self += double() would hand Python
        // that void (None) and rebind the name, so these return self.
        .def("__iadd__", [](ti::Bounds& self, double scalar) -> ti::Bounds& { self += scalar; return self; },
             "scalar"_a, nb::rv_policy::reference,
             "Shift both limits up by scalar and reclassify the bounds type")
        .def("__isub__", [](ti::Bounds& self, double scalar) -> ti::Bounds& { self -= scalar; return self; },
             "scalar"_a, nb::rv_policy::reference,
             "Shift both limits down by scalar and reclassify the bounds type");

    // ========== Component (base class) ==========
    nb::class_<ti::Component>(m, "Component")
        .def("getName", &ti::Component::getName,
             "Get the component name")
        .def("getRows", &ti::Component::getRows,
             "Get the number of rows")
        .def("getValues", &ti::Component::getValues,
             "Get the current values (variables, or constraint / cost values)")
        .def("getBounds", &ti::Component::getBounds,
             "Get the bounds, one per row");

    // ========== Variables (inherits Component) ==========
    nb::class_<ti::Variables, ti::Component>(m, "Variables")
        .def("setVariables", &ti::Variables::setVariables, "x"_a,
             "Set the variable values from a flat decision vector")
        .def("getHash", &ti::Variables::getHash,
             "Get the variable hash for caching");

    // ========== Differentiable (inherits Component) ==========
    nb::class_<ti::Differentiable, ti::Component> differentiable(m, "Differentiable");

    nb::enum_<ti::Differentiable::Mode>(differentiable, "Mode")
        .value("STACK_ROWS", ti::Differentiable::Mode::kStackRows, "One row per value")
        .value("SUM_TO_SCALAR", ti::Differentiable::Mode::kSumToScalar, "All values summed into one row");

    differentiable
        .def("getJacobian", &jacobian_checked,
             "Get the Jacobian (sparse matrix); a constraint set must be linked to variables first")
        .def("isDynamic", &ti::Differentiable::isDynamic,
             "Whether this component can change its number of rows")
        .def("isScalar", &ti::Differentiable::isScalar,
             "Whether this component is scalar or stacked")
        .def("getNonZeros", &ti::Differentiable::getNonZeros,
             "Estimated number of non-zero entries in the Jacobian")
        .def("update", &ti::Differentiable::update,
             "Recompute sizing/state for dynamic-sized components")
        .def("getCoefficients", &ti::Differentiable::getCoefficients,
             "Get the coefficient vector");

    // ========== ConstraintSet (inherits Differentiable, with Python trampoline) ==========
    nb::class_<ti::ConstraintSet, ti::Differentiable, PyConstraintSet>(m, "ConstraintSet")
        .def(nb::init<std::string, int>(), "name"_a, "n_constraints"_a,
             "Create a constraint set with name and number of constraints")
        .def("linkWithVariables", &ti::ConstraintSet::linkWithVariables, "x"_a,
             "Connect the constraint with the optimization variables")
        .def("getVariables", &PyConstraintSet::getVariables,
             "Get the linked optimization variables (protected in C++, exposed via trampoline)")
        .def("getJacobian", &jacobian_checked,
             "Get the constraint Jacobian (sparse matrix)")
        .def("update", &ti::ConstraintSet::update,
             "Recompute sizing/state for dynamic-sized components")
        .def("getCoefficients", &ti::ConstraintSet::getCoefficients,
             "Get the coefficient vector");

    // ========== Var (variable segment) ==========
    // A standalone Var (no parent Node) has no variable set: a constraint built on it raises
    // VarNotInVariableSetError when linked, and a collision constraint already when built.
    nb::class_<ti::Var>(m, "Var")
        .def("__init__",
             [](ti::Var* self, Eigen::Index index, std::string name, double value, ti::Bounds bounds) {
                 new (self) ti::Var(index, std::move(name), value, bounds);
             },
             "index"_a, "name"_a, "value"_a, "bounds"_a = ti::NoBound,
             "Create a standalone scalar variable at index of the decision vector; a value outside "
             "bounds is clamped into them")
        .def("__init__",
             [](ti::Var* self, Eigen::Index index, std::string identifier, std::vector<std::string> names,
                const Eigen::VectorXd& values, const std::vector<ti::Bounds>& bounds) {
                 new (self) ti::Var(index, std::move(identifier), std::move(names), values, bounds);
             },
             "index"_a, "identifier"_a, "names"_a, "values"_a, "bounds"_a,
             "Create a standalone vector variable starting at index of the decision vector; values "
             "outside bounds are clamped into them")
        .def("getIdentifier", &ti::Var::getIdentifier,
             "Get the identifier for the variable segment")
        .def("getParent",
             [](nb::pointer_and_handle<ti::Var> self) { return node_object(self.p->getParent(), self.h); },
             nb::sig("def getParent(self) -> Node | None"),
             "The Node holding this variable, or None for a standalone Var")
        .def("getIndex", &ti::Var::getIndex,
             "Get the starting index of this variable segment")
        .def("size", &ti::Var::size,
             "Get the number of elements in this variable segment")
        .def("value", &ti::Var::value,
             "Get the current variable values")
        .def("name", &ti::Var::name,
             "Get the variable names")
        .def("bounds", &ti::Var::bounds,
             "Get the variable bounds")
        .def("setVariables", &ti::Var::setVariables, "x"_a,
             "Set the variable values from the full decision vector");

    // ========== Node (group of related variables) ==========
    // Built on the heap, not in the Python object: NodesVariables takes Nodes by unique_ptr, and
    // nanobind can hand only a C++-allocated instance over to a unique_ptr.
    nb::class_<ti::Node>(m, "Node")
        .def(nb::new_([](std::string node_name) { return std::make_unique<ti::Node>(std::move(node_name)); }),
             "node_name"_a = "Node",
             "Create a node with an optional name")
        .def("getName", &ti::Node::getName,
             "Get the name of this node")
        .def("getParent",
             [](const ti::Node& self) -> nb::object {
                 if (self.getParent() == nullptr)
                     return nb::none();
                 std::shared_ptr<ti::NodesVariables> set = NodesVariablesBinding::parent_of(self);
                 // the set's own Python object, if it has one: a Python subclass keeps its type
                 nb::object found = nb::find(set.get());
                 if (found.is_valid() && nb::inst_ready(found))
                     return found;
                 return nb::cast(std::move(set));
             },
             nb::sig("def getParent(self) -> NodesVariables | None"),
             "The NodesVariables holding this node (kept alive by the result), or None before it is "
             "handed to one. Raises NodeParentUnavailableError for a Node of a set C++ owns (a "
             "planner's, handed to a profile's create), even during that call")
        .def("addVar",
             nb::overload_cast<const std::string&, double, ti::Bounds>(&ti::Node::addVar),
             "name"_a, "value"_a, "bounds"_a = ti::NoBound, nb::keep_alive<0, 1>(),
             "Add a scalar variable to this node")
        .def("addVar",
             nb::overload_cast<const std::string&, const std::vector<std::string>&,
                               const Eigen::VectorXd&, const std::vector<ti::Bounds>&>(&ti::Node::addVar),
             "name"_a, "child_names"_a, "values"_a, "bounds"_a, nb::keep_alive<0, 1>(),
             "Add a vector-valued variable to this node")
        .def("hasVar", &ti::Node::hasVar, "name"_a,
             "Check whether this node has a variable by name")
        .def("getVar", &ti::Node::getVar, "name"_a, nb::keep_alive<0, 1>(),
             "Get a variable by name")
        .def("getValues", &ti::Node::getValues,
             "Get all variable values as a single vector")
        .def("getBounds", &ti::Node::getBounds,
             "Get all variable bounds")
        .def("size", &ti::Node::size,
             "Get total number of scalar decision variables")
        .def("setVariables", &ti::Node::setVariables, "x"_a,
             "Update all variable values from the full decision vector");

    // ========== NodesVariables (main variable container, replaces JointPosition) ==========
    // The constructor takes each Node by unique_ptr: the Python Node objects hand over ownership
    // and become unusable (take_over_nodes). Moving the Nodes instead would leave each Var's parent
    // pointer on the moved-from Python object. Each handed-over object keeps the set alive, so a
    // Var read from the Node before the handover keeps its parent chain. The set is a
    // NodesVariablesBinding (registered for Node.getParent); Python sees NodesVariables.
    nb::class_<ti::NodesVariables, ti::Variables>(m, "NodesVariables")
        // __new__ by hand, not nb::new_: for a Python subclass, nb::new_ wraps the base-type object in
        // a second, subclass-typed one, so the set would have two Python objects and Node.getParent
        // could find the base one. Here the set gets exactly one, of the requested type.
        .def_static("__new__",
             [](nb::handle cls, std::string variable_name, nb::typed<nb::sequence, ti::Node> node_objects) {
                 // first: before any Node is taken over, as Python's own __new__ checks
                 const nb::handle base = nb::type<ti::NodesVariables>();
                 if (!PyType_Check(cls.ptr()) ||
                     !PyType_IsSubtype(reinterpret_cast<PyTypeObject*>(cls.ptr()),
                                       reinterpret_cast<PyTypeObject*>(base.ptr()))) {
                     // Python's wording: the type's name, or the object's repr if it is no type
                     const std::string name = PyType_Check(cls.ptr()) ? nb::cast<std::string>(nb::type_name(cls))
                                                                      : nb::cast<std::string>(nb::repr(cls));
                     throw nb::type_error(("NodesVariables.__new__(" + name + "): " + name +
                                           " is not a subtype of NodesVariables").c_str());
                 }
                 if (nb::isinstance<nb::str>(node_objects))
                     throw nb::type_error("NodesVariables: nodes must be a sequence of Node, not str");
                 std::vector<nb::handle> nodes;
                 for (nb::handle node : node_objects)
                     nodes.push_back(node);
                 std::shared_ptr<ti::NodesVariables> set = std::make_shared<NodesVariablesBinding>(
                     std::move(variable_name), take_over_nodes(nodes));
                 const auto hold_set = [&set](nb::handle holder) {
                     nb::detail::keep_alive(holder.ptr(), new std::shared_ptr<ti::NodesVariables>(set),
                                            [](void* p) noexcept {
                                                delete static_cast<std::shared_ptr<ti::NodesVariables>*>(p);
                                            });
                 };
                 for (const nb::handle node : nodes)
                     hold_set(node);
                 if (cls.is(nb::type<ti::NodesVariables>()))
                     return nb::cast(set);
                 // a Python subclass: one object of that type, referring to the set and holding it
                 nb::object self = nb::inst_reference(cls, set.get());
                 hold_set(self);
                 return self;
             },
             "cls"_a, "variable_name"_a, "nodes"_a)
        .def("__init__", [](nb::handle, const std::string&, const nb::typed<nb::sequence, ti::Node>&) {},
             "variable_name"_a, "nodes"_a,
             "Create the variable set from its nodes, in order; takes ownership of the nodes, which "
             "become unusable through the caller's references (read them back with getNode / "
             "getNodes). Raises NodeOwnershipError for a Node given twice or already in a set")
        .def("getNode",
             [](nb::pointer_and_handle<ti::NodesVariables> self, std::size_t opt_idx) {
                 return node_object(self.p->getNode(opt_idx).get(), self.h);
             },
             "opt_idx"_a, nb::sig("def getNode(self, opt_idx: int) -> Node"),
             "Get node based on index")
        .def("getNodes",
             [](nb::pointer_and_handle<ti::NodesVariables> self) {
                 nb::list nodes;
                 for (const auto& node : self.p->getNodes())
                     nodes.append(node_object(node.get(), self.h));
                 return nodes;
             },
             nb::sig("def getNodes(self) -> list[Node]"),
             "Get all nodes")
        .def("getDim", &ti::NodesVariables::getDim,
             "Get the dimensions of every node");

    // ========== Factory for NodesVariables ==========
    // The one-call form of the NodesVariables constructor: one Node with one vector Var
    // ("joints") per waypoint, built on the C++ side.
    m.def("createNodesVariables",
          [](const std::string& variable_name,
             const std::vector<std::string>& joint_names,
             const std::vector<Eigen::VectorXd>& initial_values,
             const std::vector<ti::Bounds>& bounds) -> std::shared_ptr<ti::NodesVariables> {
              std::vector<std::unique_ptr<ti::Node>> nodes;
              nodes.reserve(initial_values.size());
              for (size_t i = 0; i < initial_values.size(); ++i) {
                  auto node = std::make_unique<ti::Node>(variable_name + "_" + std::to_string(i));
                  node->addVar("joints", joint_names, initial_values[i], bounds);
                  nodes.push_back(std::move(node));
              }
              return std::make_shared<NodesVariablesBinding>(variable_name, std::move(nodes));
          },
          "variable_name"_a, "joint_names"_a, "initial_values"_a, "bounds"_a,
          "Create NodesVariables with one vector Var per node.\n\n"
          "Args:\n"
          "    variable_name: Name for the variable set\n"
          "    joint_names: Joint name for each DOF\n"
          "    initial_values: List of initial joint value vectors (one per waypoint)\n"
          "    bounds: Joint bounds (shared across all waypoints)");

    // ========== Utility functions ==========
    m.def("interpolate", &ti::interpolate, "start"_a, "end"_a, "steps"_a,
          "Interpolate between two joint vectors, returning 'steps' waypoints");

    m.def("toBounds", nb::overload_cast<const Eigen::Ref<const Eigen::MatrixX2d>&>(&ti::toBounds),
          "limits"_a, "Convert Nx2 matrix [lower, upper] to vector of Bounds");

    m.def("toBounds", nb::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                                         const Eigen::Ref<const Eigen::VectorXd>&>(&ti::toBounds),
          "lower_limits"_a, "upper_limits"_a, "Convert lower/upper limit vectors to vector of Bounds");

    m.def("getClosestValidPoint",
          [](const Eigen::Ref<const Eigen::VectorXd>& input, const std::vector<ti::Bounds>& bounds) {
              require_one_bound_per_value(input, bounds, "getClosestValidPoint");
              return ti::getClosestValidPoint(input, bounds);
          },
          "input"_a, "bounds"_a, "input clamped into bounds, entry by entry");

    m.def("calcBoundsErrors",
          [](const Eigen::Ref<const Eigen::VectorXd>& input, const std::vector<ti::Bounds>& bounds) {
              require_one_bound_per_value(input, bounds, "calcBoundsErrors");
              Eigen::VectorXd out(input.size());
              ti::calcBoundsErrors(out, input, bounds);
              return out;
          },
          "input"_a, "bounds"_a,
          "Signed bound error per entry: input - lower below the lower bound (negative), "
          "input - upper above the upper bound (positive), else 0");

    m.def("calcBoundsViolations",
          [](const Eigen::Ref<const Eigen::VectorXd>& input, const std::vector<ti::Bounds>& bounds) {
              require_one_bound_per_value(input, bounds, "calcBoundsViolations");
              Eigen::VectorXd out(input.size());
              ti::calcBoundsViolations(out, input, bounds);
              return out;
          },
          "input"_a, "bounds"_a, "Bound violation per entry: the magnitude of calcBoundsErrors");

    // Numerical Jacobians, to check an analytic getJacobian (a Python ConstraintSet's, say). Both
    // call back into Python (constraint_set's methods, f), so neither releases the GIL.
    m.def("calcNumericalConstraintGradient",
          [](ti::Variables& variables, ti::ConstraintSet& constraint_set, double epsilon) {
              ti::Jacobian jacobian = ti::calcNumericalConstraintGradient(variables, constraint_set, epsilon);
              jacobian.makeCompressed();  // assembled with coeffRef
              return jacobian;
          },
          "variables"_a, "constraint_set"_a, "epsilon"_a = NUMERICAL_GRADIENT_EPSILON,
          "Forward-difference Jacobian of constraint_set's values over variables (sparse matrix). "
          "Perturbs each variable in turn through variables.setVariables, calling update() and "
          "getValues(), and restores the variables afterwards.");
    m.def("calcForwardNumJac",
          [](const ti::ErrorCalculator& f, const Eigen::Ref<const Eigen::VectorXd>& x, double epsilon) {
              std::optional<Eigen::Index> rows;  // len(f(x)), from the first call
              const ti::ErrorCalculator checked = [&f, &rows](const Eigen::Ref<const Eigen::VectorXd>& at) {
                  Eigen::VectorXd values = f(at);
                  if (!rows)
                      rows = values.size();
                  else if (values.size() != *rows)
                      throw NumericalJacobianSizeError(
                          "calcForwardNumJac: f returned " + std::to_string(*rows) +
                          " values at x and " + std::to_string(values.size()) + " at a perturbed x");
                  return values;
              };
              return ti::calcForwardNumJac(checked, x, epsilon);
          },
          "f"_a, "x"_a, "epsilon"_a,
          "Forward-difference Jacobian of f at x (sparse matrix, len(f(x)) x len(x)); raises "
          "NumericalJacobianSizeError if f's output length changes");

    // ========== Trajectory conversion (trajopt_utils.h) ==========
    m.def("toTrajArray",
          [](const std::vector<std::shared_ptr<const ti::Var>>& joint_positions) {
              require_non_null(joint_positions, "toTrajArray", "joint_positions", "a Var");
              for (std::size_t i = 1; i < joint_positions.size(); ++i)
                  if (joint_positions[i]->size() != joint_positions.front()->size())
                      throw VarSizeMismatchError(
                          "toTrajArray: joint_positions[" + std::to_string(i) + "] has " +
                          std::to_string(joint_positions[i]->size()) + " values; joint_positions[0] has " +
                          std::to_string(joint_positions.front()->size()));
              return ti::toTrajArray(joint_positions);
          },
          "joint_positions"_a,
          "The Vars' values as a waypoints x joints matrix, one row per Var");
    m.def("toJointTrajectory",
          [](const std::vector<std::shared_ptr<const ti::Var>>& joint_positions) {
              return ti::toJointTrajectory(
                  require_non_null(joint_positions, "toJointTrajectory", "joint_positions", "a Var"));
          },
          "joint_positions"_a,
          "A JointTrajectory with one JointState (the Var's names and values) per Var");

    // ========== CartPosConstraint ==========
    // Constructor takes individual parameters (CartPosInfo struct removed)
    //
    // range_bound_handling defaults to kSplitToTwoInequalities, matching the C++ ctor
    // default: every row whose bounds are a true range [lb, ub] is emitted as two
    // one-sided rows, g(x) >= lb and g(x) <= ub, so the row count grows accordingly.
    // kKeepAsIs leaves such a row as a single [lb, ub] row instead - the form to pass
    // when reading back an asymmetric band. Equality and one-sided bounds are unaffected
    // either way.
    nb::class_<CartPosConstraintBinding, ti::ConstraintSet>(m, "CartPosConstraint")
        .def("__init__", [](CartPosConstraintBinding* self,
                            std::shared_ptr<const ti::Var> position_var,
                            std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                            tesseract::common::LinkId source_frame,
                            tesseract::common::LinkId target_frame,
                            const Eigen::Isometry3d& source_frame_offset,
                            const Eigen::Isometry3d& target_frame_offset,
                            const std::string& name,
                            ti::RangeBoundHandling range_bound_handling) {
                 new (self) CartPosConstraintBinding({position_var}, position_var, manip,
                     std::move(source_frame), std::move(target_frame),
                     source_frame_offset, target_frame_offset,
                     name, range_bound_handling);
             },
             "position_var"_a, "manip"_a,
             "source_frame"_a, "target_frame"_a,
             "source_frame_offset"_a, "target_frame_offset"_a,
             "name"_a = "CartPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create Cartesian position constraint")
        .def("__init__", [](CartPosConstraintBinding* self,
                            std::shared_ptr<const ti::Var> position_var,
                            const Eigen::VectorXd& coeffs,
                            const std::vector<ti::Bounds>& bounds,
                            std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                            tesseract::common::LinkId source_frame,
                            tesseract::common::LinkId target_frame,
                            const Eigen::Isometry3d& source_frame_offset,
                            const Eigen::Isometry3d& target_frame_offset,
                            const std::string& name,
                            ti::RangeBoundHandling range_bound_handling) {
                 new (self) CartPosConstraintBinding({position_var}, position_var, coeffs, bounds, manip,
                     std::move(source_frame), std::move(target_frame),
                     source_frame_offset, target_frame_offset,
                     name, range_bound_handling);
             },
             "position_var"_a, "coeffs"_a, "bounds"_a, "manip"_a,
             "source_frame"_a, "target_frame"_a,
             "source_frame_offset"_a, "target_frame_offset"_a,
             "name"_a = "CartPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create Cartesian position constraint with per-axis coefficients and "
             "bounds. coeffs and bounds are both length 6, ordered "
             "[x, y, z, rx, ry, rz]; a zero coefficient frees that axis.")
        .def("calcValues", &ti::CartPosConstraint::calcValues, "joint_vals"_a,
             "Calculate error values for given joint values")
        .def("calcJacobianBlock",
             [](const CartPosConstraintBinding& self, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 // CartPosConstraint does not expose its manipulator; its Var holds one value per joint
                 return jacobian_block_checked(self, joint_vals, self.vars().front()->size());
             },
             "joint_vals"_a,
             "The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); "
             "needs linked variables")
        .def("getValues", &ti::CartPosConstraint::getValues,
             "Get current constraint values")
        .def("setTargetPose", &ti::CartPosConstraint::setTargetPose, "target_frame_offset"_a,
             "Set the target pose - critical for online planning!")
        .def("getTargetPose", &ti::CartPosConstraint::getTargetPose,
             "Get the target pose for the constraint")
        .def("getCurrentPose", &ti::CartPosConstraint::getCurrentPose,
             "Get current TCP pose in world frame")
        .def_rw("use_numeric_differentiation", &ti::CartPosConstraint::use_numeric_differentiation,
                "If true, use numeric differentiation (default: true)");

    // ========== JointPosConstraint ==========
    nb::class_<JointPosConstraintBinding, ti::ConstraintSet>(m, "JointPosConstraint")
        .def("__init__",
             [](JointPosConstraintBinding* self, const Eigen::VectorXd& target,
                const std::shared_ptr<const ti::Var>& position_var, const Eigen::VectorXd& coeffs,
                std::string name, ti::RangeBoundHandling range_bound_handling) {
                 new (self) JointPosConstraintBinding({position_var}, target, position_var, coeffs,
                                                      std::move(name), range_bound_handling);
             },
             "target"_a, "position_var"_a, "coeffs"_a, "name"_a = "JointPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create joint position constraint with target values")
        // trajopt#592 broadcasts length-0/1 coeffs before the range split; the binding-side
        // broadcast trajopt 0.35.0 needed (tesseract_nanobind#146) is gone on this branch.
        .def("__init__", [](JointPosConstraintBinding* self,
                            const std::vector<ti::Bounds>& bounds,
                            const std::shared_ptr<const ti::Var>& position_var,
                            const Eigen::VectorXd& coeffs,
                            std::string name,
                            ti::RangeBoundHandling range_bound_handling) {
                 new (self) JointPosConstraintBinding({position_var}, bounds, position_var, coeffs,
                                                      std::move(name), range_bound_handling);
             },
             "bounds"_a, "position_var"_a, "coeffs"_a, "name"_a = "JointPos",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             "Create joint position constraint with one bound per joint: an equality, a "
             "one-sided limit or a range. coeffs has length n_dof, 1 (one weight for every "
             "joint) or 0 (weight 1 for every joint). A range becomes two one-sided rows "
             "unless range_bound_handling is KEEP_AS_IS.")
        .def("getValues", &ti::JointPosConstraint::getValues,
             "Get current constraint values");

    // ========== JointVelConstraint ==========
    nb::class_<JointVelConstraintBinding, ti::ConstraintSet>(m, "JointVelConstraint")
        .def("__init__",
             [](JointVelConstraintBinding* self, const Eigen::VectorXd& targets,
                const std::vector<std::shared_ptr<const ti::Var>>& position_vars,
                const Eigen::VectorXd& coeffs, std::string name) {
                 new (self) JointVelConstraintBinding(
                     position_vars, targets, require_non_null(position_vars, "JointVelConstraint", "position_vars", "a Var"),
                     coeffs, std::move(name));
             },
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointVel",
             "Create joint velocity constraint with target values")
        .def("__init__", joint_diff_bounds_init<JointVelConstraintBinding>("JointVelConstraint"),
             "bounds"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointVel",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             JOINT_DIFF_BOUNDS_DOC)
        .def("getValues", &ti::JointVelConstraint::getValues,
             "Get current constraint values");

    // ========== JointAccelConstraint ==========
    nb::class_<JointAccelConstraintBinding, ti::ConstraintSet>(m, "JointAccelConstraint")
        .def("__init__",
             [](JointAccelConstraintBinding* self, const Eigen::VectorXd& targets,
                const std::vector<std::shared_ptr<const ti::Var>>& position_vars,
                const Eigen::VectorXd& coeffs, std::string name) {
                 new (self) JointAccelConstraintBinding(
                     position_vars, targets, require_non_null(position_vars, "JointAccelConstraint", "position_vars", "a Var"),
                     coeffs, std::move(name));
             },
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointAccel",
             "Create joint acceleration constraint with target values")
        .def("__init__", joint_diff_bounds_init<JointAccelConstraintBinding>("JointAccelConstraint"),
             "bounds"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointAccel",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             JOINT_DIFF_BOUNDS_DOC)
        .def("getValues", &ti::JointAccelConstraint::getValues,
             "Get current constraint values");

    bind_collision_terms(m);  // trajopt_ifopt_collision.cpp

    // ========== JointJerkConstraint ==========
    nb::class_<JointJerkConstraintBinding, ti::ConstraintSet>(m, "JointJerkConstraint")
        .def("__init__",
             [](JointJerkConstraintBinding* self, const Eigen::VectorXd& targets,
                const std::vector<std::shared_ptr<const ti::Var>>& position_vars,
                const Eigen::VectorXd& coeffs, std::string name) {
                 new (self) JointJerkConstraintBinding(
                     position_vars, targets, require_non_null(position_vars, "JointJerkConstraint", "position_vars", "a Var"),
                     coeffs, std::move(name));
             },
             "targets"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointJerk",
             "Create joint jerk constraint (requires 4+ consecutive waypoints)")
        .def("__init__", joint_diff_bounds_init<JointJerkConstraintBinding>("JointJerkConstraint"),
             "bounds"_a, "position_vars"_a, "coeffs"_a, "name"_a = "JointJerk",
             "range_bound_handling"_a = ti::RangeBoundHandling::kSplitToTwoInequalities,
             JOINT_DIFF_BOUNDS_DOC)
        .def("getValues", &ti::JointJerkConstraint::getValues,
             "Get current constraint values");

    // ========== CartLineInfo (struct) ==========
    // The C++ defaults of the trailing Eigen parameters are spelled as overloads (file header note)
    using JointGroupConstPtr = std::shared_ptr<const tesseract::kinematics::JointGroup>;
    nb::class_<ti::CartLineInfo>(m, "CartLineInfo")
        .def(nb::init<>())
        .def(nb::init<JointGroupConstPtr, tesseract::common::LinkId, tesseract::common::LinkId, const Eigen::Isometry3d&,
                      const Eigen::Isometry3d&, const Eigen::Isometry3d&, const Eigen::VectorXi&>(),
             "manip"_a, "source_frame"_a, "target_frame"_a, "target_frame_offset1"_a,
             "target_frame_offset2"_a, "source_frame_offset"_a, "indices"_a,
             "Create the info of a line from target_frame_offset1 to target_frame_offset2")
        .def("__init__",
             [](ti::CartLineInfo* self, JointGroupConstPtr manip, tesseract::common::LinkId source_frame,
                tesseract::common::LinkId target_frame, const Eigen::Isometry3d& target_frame_offset1,
                const Eigen::Isometry3d& target_frame_offset2, const Eigen::Isometry3d& source_frame_offset) {
                 new (self) ti::CartLineInfo(std::move(manip), std::move(source_frame), std::move(target_frame),
                                             target_frame_offset1, target_frame_offset2, source_frame_offset);
             },
             "manip"_a, "source_frame"_a, "target_frame"_a, "target_frame_offset1"_a,
             "target_frame_offset2"_a, "source_frame_offset"_a,
             "Create the info of a line, constraining all 6 indices")
        .def("__init__",
             [](ti::CartLineInfo* self, JointGroupConstPtr manip, tesseract::common::LinkId source_frame,
                tesseract::common::LinkId target_frame, const Eigen::Isometry3d& target_frame_offset1,
                const Eigen::Isometry3d& target_frame_offset2) {
                 new (self) ti::CartLineInfo(std::move(manip), std::move(source_frame), std::move(target_frame),
                                             target_frame_offset1, target_frame_offset2);
             },
             "manip"_a, "source_frame"_a, "target_frame"_a, "target_frame_offset1"_a,
             "target_frame_offset2"_a,
             "Create the info of a line, with an identity source_frame_offset and all 6 indices")
        .def_rw("manip", &ti::CartLineInfo::manip, "The joint group")
        .def_rw("source_frame", &ti::CartLineInfo::source_frame, "TCP frame", nb::rv_policy::copy)
        .def_rw("target_frame", &ti::CartLineInfo::target_frame, "Reference frame", nb::rv_policy::copy)
        .def_rw("source_frame_offset", &ti::CartLineInfo::source_frame_offset, "TCP offset")
        .def_rw("target_frame_offset1", &ti::CartLineInfo::target_frame_offset1, "Line start pose")
        .def_rw("target_frame_offset2", &ti::CartLineInfo::target_frame_offset2, "Line end pose")
        .def_rw("indices", &ti::CartLineInfo::indices,
                "DOF indices to constrain: default {0,1,2,3,4,5}");

    // ========== CartLineConstraint ==========
    nb::class_<CartLineConstraintBinding, ti::ConstraintSet>(m, "CartLineConstraint")
        .def("__init__",
             [](CartLineConstraintBinding* self, ti::CartLineInfo info, std::shared_ptr<const ti::Var> position_var,
                const Eigen::VectorXd& coeffs, std::string name) {
                 new (self) CartLineConstraintBinding({position_var}, std::move(info), position_var, coeffs,
                                                      std::move(name));
             },
             "info"_a, "position_var"_a, "coeffs"_a, "name"_a = "CartLine",
             "Create Cartesian line constraint")
        .def("getValues", &ti::CartLineConstraint::getValues,
             "Get current constraint values")
        .def("setBounds", &set_bounds_checked<CartLineConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getLine", &ti::CartLineConstraint::getLine,
             "The line's two end poses (target_frame_offset1, target_frame_offset2)")
        .def("getInfo", &ti::CartLineConstraint::getInfo, nb::rv_policy::reference_internal,
             "The constraint's CartLineInfo")
        .def("getJacobian", [](const CartLineConstraintBinding& self) {
                 require_linked(self);
                 return jacobian_checked(self);
             },
             "Get the constraint Jacobian (sparse matrix), one row per index")
        .def("calcJacobianBlock",
             [](const CartLineConstraintBinding& self, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 return jacobian_block_checked(self, joint_vals, self.getInfo().manip->numJoints());
             },
             "joint_vals"_a,
             "The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); "
             "needs linked variables")
        .def("calcValues", &ti::CartLineConstraint::calcValues, "joint_vals"_a,
             "Calculate error values for given joint values")
        .def_static("getLinePoint", &ti::CartLineConstraint::getLinePoint,
             "source_tf"_a, "target_tf1"_a, "target_tf2"_a,
             "Find nearest point on line (uses SLERP for orientation)")
        .def_rw("use_numeric_differentiation", &ti::CartLineConstraint::use_numeric_differentiation,
                "If true, use numeric differentiation (default: true)");

    // ========== CartAxisKinematics ==========
    // The axis map both CartAxis constraints share: v = R_target^T R_source a.
    auto cart_axis_kinematics = nb::class_<ti::CartAxisKinematics>(m, "CartAxisKinematics",
        "A source link axis expressed in a target link's coordinates, v(q) = R_target(q)^T R_source(q) a,\n"
        "and its 3 x n Jacobian.")
        .def("__init__", [](ti::CartAxisKinematics* self, std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                            tesseract::common::LinkId source_frame, const Eigen::Vector3d& source_axis,
                            tesseract::common::LinkId target_frame) {
                 new (self) ti::CartAxisKinematics(std::move(manip), std::move(source_frame),
                                                   require_axis(source_axis, "source axis"), std::move(target_frame));
             },
             "manip"_a, "source_frame"_a, "source_axis"_a, "target_frame"_a,
             "source_axis is in source link coordinates and normalized. Raises on a link that is not in\n"
             "manip or an invalid axis.")
        .def("calcAxis", &ti::CartAxisKinematics::calcAxis, "joint_vals"_a, "v at joint_vals")
        .def("calcAxisAndJacobian",
             [](const ti::CartAxisKinematics& self, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 Eigen::Vector3d axis;
                 Eigen::Matrix3Xd axis_jacobian;
                 self.calcAxisAndJacobian(joint_vals, axis, axis_jacobian);
                 return std::make_tuple(axis, axis_jacobian);
             },
             "joint_vals"_a, "(v, dv/dq) at joint_vals; dv/dq is 3 x numJoints()")
        .def("numJoints", &ti::CartAxisKinematics::numJoints)
        .def("getSourceAxis", &ti::CartAxisKinematics::getSourceAxis, "The unit source axis, in source link coordinates")
        .def_static("mostReachableDirection", &ti::CartAxisKinematics::mostReachableDirection, "tangent_jacobian"_a,
                    "The unit 2D direction the 2 x n tangent Jacobian moves along most per unit joint step")
        .def_static("toUnitAxis", [](const Eigen::Vector3d& axis, const std::string& what) {
                        return ti::CartAxisKinematics::toUnitAxis(require_axis(axis, what), what);
                    }, "axis"_a, "what"_a,
                    "axis normalized; raises DegenerateAxisError (a ValueError) naming `what` if it is zero or "
                    "not finite");
    // A class constant, not a static property: stubgen then renders its value, not a getter signature.
    cart_axis_kinematics.attr("SIN_ANGLE_FLOOR") = ti::CartAxisKinematics::SIN_ANGLE_FLOOR;

    // ========== CartAxisConeConstraint (trajopt#602) ==========
    nb::class_<CartAxisConeConstraintBinding, ti::ConstraintSet>(m, "CartAxisConeConstraint",
        "Keeps the angle between a source link axis and a target link axis at most half_angle: one\n"
        "one-sided row, phi(q) = atan2(|a_tgt x v|, a_tgt . v) <= half_angle, in radians. Rotation about\n"
        "the source axis stays free. For half_angle near 0 use CartAxisAlignConstraint.")
        .def("__init__",
             [](CartAxisConeConstraintBinding* self, std::shared_ptr<const ti::Var> position_var,
                std::shared_ptr<const tesseract::kinematics::JointGroup> manip, tesseract::common::LinkId source_frame,
                const Eigen::Vector3d& source_axis, tesseract::common::LinkId target_frame,
                const Eigen::Vector3d& target_axis, double half_angle, double coeff, std::string name) {
                 require_axis(source_axis, "source axis");
                 require_axis(target_axis, "target axis");
                 new (self) CartAxisConeConstraintBinding({position_var}, position_var, std::move(manip),
                                                          std::move(source_frame), source_axis,
                                                          std::move(target_frame), target_axis, half_angle, coeff,
                                                          std::move(name));
             },
             "position_var"_a, "manip"_a, "source_frame"_a, "source_axis"_a, "target_frame"_a, "target_axis"_a,
             "half_angle"_a, "coeff"_a = 1.0, "name"_a = "CartAxisCone",
             "Axes are in their links' coordinates and normalized; 0 < half_angle < pi. Raises on an invalid\n"
             "link, axis, half angle or coefficient.")
        .def("calcValues", &ti::CartAxisConeConstraint::calcValues, "joint_vals"_a,
             "The angle phi between the axes for the given joint values")
        .def("calcJacobianBlock",
             [](const CartAxisConeConstraintBinding& self, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 return jacobian_block_checked(self, joint_vals, self.vars().front()->size());
             },
             "joint_vals"_a,
             "The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); "
             "needs linked variables")
        .def("setTargetAxis", [](CartAxisConeConstraintBinding& self, const Eigen::Vector3d& target_axis) {
            self.setTargetAxis(require_axis(target_axis, "target axis"));
        }, "target_axis"_a,
             "Set the cone axis, in target link coordinates")
        .def("getTargetAxis", &ti::CartAxisConeConstraint::getTargetAxis, "The cone axis, normalized")
        .def("getHalfAngle", &ti::CartAxisConeConstraint::getHalfAngle, "The cone half angle, in radians");

    // ========== CartAxisAlignConstraint (trajopt#602) ==========
    nb::class_<CartAxisAlignConstraintBinding, ti::ConstraintSet>(m, "CartAxisAlignConstraint",
        "Points a source link axis along a target link axis; rotation about the axis is free. Two\n"
        "equality rows, in radians: the tilt angle times the unit tilt direction, zero only when the\n"
        "axes coincide.")
        .def("__init__",
             [](CartAxisAlignConstraintBinding* self, std::shared_ptr<const ti::Var> position_var,
                std::shared_ptr<const tesseract::kinematics::JointGroup> manip, tesseract::common::LinkId source_frame,
                const Eigen::Vector3d& source_axis, tesseract::common::LinkId target_frame,
                const Eigen::Vector3d& target_axis, double coeff, std::string name) {
                 require_axis(source_axis, "source axis");
                 require_axis(target_axis, "target axis");
                 new (self) CartAxisAlignConstraintBinding({position_var}, position_var, std::move(manip),
                                                           std::move(source_frame), source_axis,
                                                           std::move(target_frame), target_axis, coeff,
                                                           std::move(name));
             },
             "position_var"_a, "manip"_a, "source_frame"_a, "source_axis"_a, "target_frame"_a, "target_axis"_a,
             "coeff"_a = 1.0, "name"_a = "CartAxisAlign",
             "Axes are in their links' coordinates and normalized. Raises on an invalid link, axis or\n"
             "coefficient.")
        .def("calcValues", &ti::CartAxisAlignConstraint::calcValues, "joint_vals"_a,
             "The two tilt rows for the given joint values")
        .def("calcJacobianBlock",
             [](const CartAxisAlignConstraintBinding& self, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 return jacobian_block_checked(self, joint_vals, self.vars().front()->size());
             },
             "joint_vals"_a,
             "The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); "
             "needs linked variables")
        .def("setTargetAxis", [](CartAxisAlignConstraintBinding& self, const Eigen::Vector3d& target_axis) {
            self.setTargetAxis(require_axis(target_axis, "target axis"));
        }, "target_axis"_a,
             "Set the axis to align with, in target link coordinates")
        .def("getTargetAxis", &ti::CartAxisAlignConstraint::getTargetAxis, "The axis to align with, normalized");

    // ========== InverseKinematicsInfo (struct) ==========
    using KinematicGroupConstPtr = std::shared_ptr<const tesseract::kinematics::KinematicGroup>;
    nb::class_<ti::InverseKinematicsInfo>(m, "InverseKinematicsInfo")
        .def(nb::init<>())
        .def(nb::init<KinematicGroupConstPtr, tesseract::common::LinkId, tesseract::common::LinkId, const Eigen::Isometry3d&>(),
             "manip"_a, "working_frame"_a, "tcp_frame"_a, "tcp_offset"_a,
             "Create IK info; raises if manip has no link tcp_frame")
        .def("__init__",
             [](ti::InverseKinematicsInfo* self, KinematicGroupConstPtr manip, tesseract::common::LinkId working_frame,
                tesseract::common::LinkId tcp_frame) {
                 new (self) ti::InverseKinematicsInfo(std::move(manip), std::move(working_frame),
                                                      std::move(tcp_frame));
             },
             "manip"_a, "working_frame"_a, "tcp_frame"_a,
             "Create IK info with an identity tcp_offset; raises if manip has no link tcp_frame")
        .def_rw("manip", &ti::InverseKinematicsInfo::manip, "The kinematic group (with IK solver)")
        .def_rw("working_frame", &ti::InverseKinematicsInfo::working_frame, "Working frame (not currently used)", nb::rv_policy::copy)
        .def_rw("tcp_frame", &ti::InverseKinematicsInfo::tcp_frame, "TCP frame (not currently used)", nb::rv_policy::copy)
        .def_rw("tcp_offset", &ti::InverseKinematicsInfo::tcp_offset, "TCP offset (not currently used)");

    // ========== InverseKinematicsConstraint ==========
    nb::class_<InverseKinematicsConstraintBinding, ti::ConstraintSet>(m, "InverseKinematicsConstraint")
        .def("__init__",
             [](InverseKinematicsConstraintBinding* self, const Eigen::Isometry3d& target_pose,
                ti::InverseKinematicsInfo::ConstPtr kinematic_info, std::shared_ptr<const ti::Var> constraint_var,
                std::shared_ptr<const ti::Var> seed_var, std::string name) {
                 new (self) InverseKinematicsConstraintBinding({constraint_var, seed_var}, target_pose,
                                                               std::move(kinematic_info), constraint_var,
                                                               seed_var, std::move(name));
             },
             "target_pose"_a, "kinematic_info"_a, "constraint_var"_a, "seed_var"_a,
             "name"_a = "InverseKinematics",
             "Create IK constraint (constraint_var constrained to IK solution seeded from seed_var)")
        .def("getValues", &ti::InverseKinematicsConstraint::getValues,
             "Get current constraint values")
        .def("setBounds", &set_bounds_checked<InverseKinematicsConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("calcJacobianBlock",
             // getKinematicInfo() is non-const upstream, so self is taken by non-const reference
             [](InverseKinematicsConstraintBinding& self,
                const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 return jacobian_block_checked(self, joint_vals,
                                               self.getKinematicInfo()->manip->numJoints());
             },
             "joint_vals"_a,
             "The constraint Jacobian at joint_vals, the constraint var's values (sparse matrix, "
             "sized like getJacobian()); needs linked variables")
        .def("setTargetPose", &ti::InverseKinematicsConstraint::setTargetPose, "target_pose"_a,
             "Set the target pose the IK solution is computed for")
        .def("getKinematicInfo", &ti::InverseKinematicsConstraint::getKinematicInfo,
             nb::rv_policy::reference_internal,
             "The constraint's InverseKinematicsInfo");
}
