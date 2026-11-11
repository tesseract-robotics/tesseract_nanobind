/**
 * @file trajopt_ifopt_collision.cpp
 * @brief trajopt_ifopt: collision evaluators, collision constraints and trajopt_common's collision data
 */

#include "trajopt_ifopt_bindings.h"

void bind_collision_terms(nb::module_& m) {
    // ========== trajopt_common::CollisionCoeffData ==========
    nb::class_<tc::CollisionCoeffData> collision_coeff_data(m, "CollisionCoeffData");
    bind_value_equality(collision_coeff_data);
    collision_coeff_data
        .def(nb::init<>())
        .def(nb::init<double>(), "default_collision_coeff"_a)
        .def("setDefaultCollisionCoeff", &tc::CollisionCoeffData::setDefaultCollisionCoeff)
        .def("getDefaultCollisionCoeff", &tc::CollisionCoeffData::getDefaultCollisionCoeff)
        .def("setCollisionCoeff",
             nb::overload_cast<const tesseract::common::LinkId&, const tesseract::common::LinkId&, double>(
                 &tc::CollisionCoeffData::setCollisionCoeff),
             "obj1"_a, "obj2"_a, "collision_coeff"_a)
        .def("setCollisionCoeff",
             nb::overload_cast<const tesseract::common::LinkIdPair&, double>(&tc::CollisionCoeffData::setCollisionCoeff),
             "pair"_a, "collision_coeff"_a)
        .def("getCollisionCoeff", &tc::CollisionCoeffData::getCollisionCoeff, "pair"_a)
        .def("getCollisionCoeffPairData", &tc::CollisionCoeffData::getCollisionCoeffPairData,
             "Every pair with a coefficient: {LinkIdPair: coeff}")
        .def("hasZeroCoeff", &tc::CollisionCoeffData::hasZeroCoeff, "pair"_a,
             "Whether the pair's coefficient is 0 (collision checking switched off for it)")
        .def("getPairsWithZeroCoeff", &tc::CollisionCoeffData::getPairsWithZeroCoeff,
             "The pairs whose coefficient is 0 (collision checking switched off)");

    // ========== trajopt_common::TrajOptCollisionConfig ==========
    nb::class_<tc::TrajOptCollisionConfig> trajopt_collision_config(m, "TrajOptCollisionConfig");
    bind_value_equality(trajopt_collision_config);
    trajopt_collision_config
        .def(nb::init<>())
        .def(nb::init<double, double, tesseract::collision::ContactRequest,
                      tesseract::collision::CollisionEvaluatorType, double,
                      tesseract::collision::CollisionCheckProgramType>(),
             "margin"_a, "coeff"_a, "request"_a = tesseract::collision::ContactRequest(),
             "type"_a = tesseract::collision::CollisionEvaluatorType::DISCRETE, "longest_valid_segment_length"_a = LONGEST_VALID_SEGMENT_LENGTH_DEFAULT,
             "check_program_mode"_a = tesseract::collision::CollisionCheckProgramType::ALL)
        .def_rw("enabled", &tc::TrajOptCollisionConfig::enabled)
        .def_rw("contact_manager_config", &tc::TrajOptCollisionConfig::contact_manager_config)
        .def_rw("collision_check_config", &tc::TrajOptCollisionConfig::collision_check_config)
        .def_rw("collision_coeff_data", &tc::TrajOptCollisionConfig::collision_coeff_data)
        .def_rw("collision_margin_buffer", &tc::TrajOptCollisionConfig::collision_margin_buffer)
        .def_rw("max_num_cnt", &tc::TrajOptCollisionConfig::max_num_cnt);

    // ========== trajopt_common collision results (what the evaluators compute) ==========
    nb::class_<tc::LinkGradientResults>(m, "LinkGradientResults")
        .def(nb::init<>())
        .def_rw("has_gradient", &tc::LinkGradientResults::has_gradient)
        .def_rw("gradient", &tc::LinkGradientResults::gradient)
        .def_rw("translation_vector", &tc::LinkGradientResults::translation_vector)
        .def_rw("jacobian", &tc::LinkGradientResults::jacobian)
        .def_rw("scale", &tc::LinkGradientResults::scale)
        .def_rw("cc_type", &tc::LinkGradientResults::cc_type)
        .def("clear", &tc::LinkGradientResults::clear);

    nb::class_<tc::GradientResults>(m, "GradientResults")
        .def(nb::init<>())
        .def_rw("gradients", &tc::GradientResults::gradients)
        .def_rw("cc_gradients", &tc::GradientResults::cc_gradients)
        .def_rw("error", &tc::GradientResults::error)
        .def_rw("error_with_buffer", &tc::GradientResults::error_with_buffer)
        .def("clear", &tc::GradientResults::clear);

    nb::class_<tc::LinkMaxError>(m, "LinkMaxError")
        .def(nb::init<>())
        .def_rw("has_error", &tc::LinkMaxError::has_error)
        .def_rw("error", &tc::LinkMaxError::error)
        .def_rw("error_with_buffer", &tc::LinkMaxError::error_with_buffer)
        .def("getMaxError", &tc::LinkMaxError::getMaxError)
        .def("getMaxErrorWithBuffer", &tc::LinkMaxError::getMaxErrorWithBuffer);

    nb::class_<tc::GradientResultsSet>(m, "GradientResultsSet")
        .def(nb::init<>())
        .def(nb::init<std::size_t>(), "reserve"_a)
        .def_rw("key", &tc::GradientResultsSet::key)
        .def_rw("shape_key", &tc::GradientResultsSet::shape_key)
        .def_rw("coeff", &tc::GradientResultsSet::coeff)
        .def_rw("is_continuous", &tc::GradientResultsSet::is_continuous)
        .def_rw("max_error", &tc::GradientResultsSet::max_error)
        .def_rw("results", &tc::GradientResultsSet::results)
        .def("add", &tc::GradientResultsSet::add, "gradient_result"_a)
        .def("getMaxError", &tc::GradientResultsSet::getMaxError)
        .def("getMaxErrorT0", &tc::GradientResultsSet::getMaxErrorT0)
        .def("getMaxErrorT1", &tc::GradientResultsSet::getMaxErrorT1)
        .def("getMaxErrorWithBuffer", &tc::GradientResultsSet::getMaxErrorWithBuffer)
        .def("getMaxErrorWithBufferT0", &tc::GradientResultsSet::getMaxErrorWithBufferT0)
        .def("getMaxErrorWithBufferT1", &tc::GradientResultsSet::getMaxErrorWithBufferT1);

    nb::class_<tc::CollisionCacheData>(m, "CollisionCacheData")
        .def(nb::init<>())
        .def_rw("contact_results_map", &tc::CollisionCacheData::contact_results_map)
        .def_rw("gradient_results_sets", &tc::CollisionCacheData::gradient_results_sets);

    // ========== trajopt_common::getGradient ==========
    // 0.36 moved the per-contact gradient off the evaluators (getGradient / calcGradientData) to
    // these free functions (collision_utils.h:69, :84). The C++ out-param is the return value.
    m.def("getGradient",
          [](const Eigen::VectorXd& dof_vals, const tesseract::collision::ContactResult& contact_result,
             double margin, double margin_buffer, const tesseract::kinematics::JointGroup& manip) {
              tc::GradientResults results;
              tc::getGradient(results, dof_vals, contact_result, margin, margin_buffer, manip);
              return results;
          },
          "dof_vals"_a, "contact_result"_a, "margin"_a, "margin_buffer"_a, "manip"_a,
          "Gradient of one discrete contact at `dof_vals`.");
    m.def("getGradient",
          [](const Eigen::VectorXd& dof_vals0, const Eigen::VectorXd& dof_vals1,
             const tesseract::collision::ContactResult& contact_result, double margin, double margin_buffer,
             const tesseract::kinematics::JointGroup& manip) {
              tc::GradientResults results;
              tc::getGradient(results, dof_vals0, dof_vals1, contact_result, margin, margin_buffer, manip);
              return results;
          },
          "dof_vals0"_a, "dof_vals1"_a, "contact_result"_a, "margin"_a, "margin_buffer"_a, "manip"_a,
          "Gradient of one continuous contact between `dof_vals0` and `dof_vals1`.");

    // ========== DiscreteCollisionEvaluator (base class) ==========
    // The queries are bound here only; virtual dispatch serves every evaluator. getJointGroup
    // returns a reference to the evaluator's own JointGroup, so it keeps the evaluator alive.
    nb::class_<ti::DiscreteCollisionEvaluator>(m, "DiscreteCollisionEvaluator")
        .def("calcCollisions",
             [](ti::DiscreteCollisionEvaluator& self, const Eigen::Ref<const Eigen::VectorXd>& dof_vals,
                std::size_t max_allowed) {
                 tc::CollisionCacheData collision_data;
                 self.calcCollisions(collision_data, dof_vals, max_allowed);
                 return collision_data;
             },
             "dof_vals"_a, "max_allowed"_a,
             "Check collisions at dof_vals, keeping at most max_allowed contacts per link pair; "
             "returns the contacts and their gradient result sets")
        .def("getJointGroup", &ti::DiscreteCollisionEvaluator::getJointGroup,
             nb::rv_policy::reference_internal)
        .def("getCollisionMarginBuffer", &ti::DiscreteCollisionEvaluator::getCollisionMarginBuffer)
        .def("getCollisionMarginData", &ti::DiscreteCollisionEvaluator::getCollisionMarginData,
             nb::rv_policy::reference_internal)
        .def("getCollisionCoeffData", &ti::DiscreteCollisionEvaluator::getCollisionCoeffData,
             nb::rv_policy::reference_internal);

    // ========== SingleTimestepCollisionEvaluator ==========
    // CollisionCache parameter removed in 0.34
    nb::class_<ti::SingleTimestepCollisionEvaluator, ti::DiscreteCollisionEvaluator>(m, "SingleTimestepCollisionEvaluator")
        .def(nb::init<std::shared_ptr<const tesseract::kinematics::JointGroup>,
                      std::shared_ptr<const tesseract::environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false, nb::call_guard<nb::gil_scoped_release>());

    // ========== DiscreteCollisionConstraint ==========
    nb::class_<DiscreteCollisionConstraintBinding, ti::ConstraintSet>(m, "DiscreteCollisionConstraint")
        .def("__init__", [](DiscreteCollisionConstraintBinding* self,
                            std::shared_ptr<ti::DiscreteCollisionEvaluator> eval,
                            const std::shared_ptr<const ti::Var>& var,
                            int max_num_cnt, bool fixed_sparsity, std::string name) {
                 new (self) DiscreteCollisionConstraintBinding(
                     {var}, std::move(eval), require_in_variable_set(var, "DiscreteCollisionConstraint", "position_var"),
                     max_num_cnt, fixed_sparsity, std::move(name));
             },
             "collision_evaluator"_a, "position_var"_a,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "DiscreteCollision")
        .def("getValues", &ti::DiscreteCollisionConstraint::getValues)
        .def("setBounds", &set_bounds_checked<DiscreteCollisionConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::DiscreteCollisionConstraint::getCollisionEvaluator);

    // ========== ContinuousCollisionEvaluator (base class) ==========
    nb::class_<ti::ContinuousCollisionEvaluator>(m, "ContinuousCollisionEvaluator")
        .def("calcCollisionData",
             [](ti::ContinuousCollisionEvaluator& self, const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                const Eigen::Ref<const Eigen::VectorXd>& dof_vals1, bool vars0_fixed, bool vars1_fixed,
                std::size_t max_allowed) {
                 tc::CollisionCacheData collision_data;
                 self.calcCollisionData(collision_data, dof_vals0, dof_vals1, vars0_fixed, vars1_fixed,
                                        max_allowed);
                 return collision_data;
             },
             "dof_vals0"_a, "dof_vals1"_a, "vars0_fixed"_a, "vars1_fixed"_a, "max_allowed"_a,
             "Check collisions along the segment dof_vals0 -> dof_vals1, keeping at most max_allowed "
             "contacts per link pair; returns the contacts and their gradient result sets")
        .def("getJointGroup", &ti::ContinuousCollisionEvaluator::getJointGroup,
             nb::rv_policy::reference_internal)
        .def("getCollisionMarginBuffer", &ti::ContinuousCollisionEvaluator::getCollisionMarginBuffer)
        .def("getCollisionMarginData", &ti::ContinuousCollisionEvaluator::getCollisionMarginData,
             nb::rv_policy::reference_internal)
        .def("getCollisionCoeffData", &ti::ContinuousCollisionEvaluator::getCollisionCoeffData,
             nb::rv_policy::reference_internal);

    // ========== LVSDiscreteCollisionEvaluator ==========
    // CollisionCache parameter removed in 0.34
    nb::class_<ti::LVSDiscreteCollisionEvaluator, ti::ContinuousCollisionEvaluator>(m, "LVSDiscreteCollisionEvaluator")
        .def(nb::init<std::shared_ptr<const tesseract::kinematics::JointGroup>,
                      std::shared_ptr<const tesseract::environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false, nb::call_guard<nb::gil_scoped_release>());

    // ========== LVSContinuousCollisionEvaluator ==========
    // CollisionCache parameter removed in 0.34
    nb::class_<ti::LVSContinuousCollisionEvaluator, ti::ContinuousCollisionEvaluator>(m, "LVSContinuousCollisionEvaluator")
        .def(nb::init<std::shared_ptr<const tesseract::kinematics::JointGroup>,
                      std::shared_ptr<const tesseract::environment::Environment>,
                      const tc::TrajOptCollisionConfig&,
                      bool>(),
             "manip"_a, "env"_a, "collision_config"_a,
             "dynamic_environment"_a = false, nb::call_guard<nb::gil_scoped_release>());

    // ========== ContinuousCollisionConstraint ==========
    nb::class_<ContinuousCollisionConstraintBinding, ti::ConstraintSet>(m, "ContinuousCollisionConstraint")
        .def("__init__", [](ContinuousCollisionConstraintBinding* self,
                            std::shared_ptr<ti::ContinuousCollisionEvaluator> eval,
                            std::shared_ptr<const ti::Var> var0,
                            std::shared_ptr<const ti::Var> var1,
                            bool fixed0, bool fixed1,
                            int max_num_cnt, bool fixed_sparsity, const std::string& name) {
                 std::array<std::shared_ptr<const ti::Var>, 2> vars = {
                     require_in_variable_set(var0, "ContinuousCollisionConstraint", "position_var0"),
                     require_in_variable_set(var1, "ContinuousCollisionConstraint", "position_var1")};
                 new (self) ContinuousCollisionConstraintBinding({vars[0], vars[1]}, eval, vars, fixed0, fixed1, max_num_cnt, fixed_sparsity, name);
             },
             "collision_evaluator"_a, "position_var0"_a, "position_var1"_a,
             "fixed0"_a = false, "fixed1"_a = false,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "LVSCollision")
        .def("getValues", &ti::ContinuousCollisionConstraint::getValues)
        .def("setBounds", &set_bounds_checked<ContinuousCollisionConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::ContinuousCollisionConstraint::getCollisionEvaluator);

    // ========== DiscreteCollisionConstraintD ==========
    // The TrajOpt-Ifopt planner's discrete collision term: one row per contact, recounted by
    // update(), so it has no max_num_cnt / fixed_sparsity.
    nb::class_<DiscreteCollisionConstraintDBinding, ti::ConstraintSet>(m, "DiscreteCollisionConstraintD")
        .def("__init__", [](DiscreteCollisionConstraintDBinding* self,
                            std::shared_ptr<ti::DiscreteCollisionEvaluator> eval,
                            const std::shared_ptr<const ti::Var>& var, std::string name) {
                 new (self) DiscreteCollisionConstraintDBinding(
                     {var}, std::move(eval), require_in_variable_set(var, "DiscreteCollisionConstraintD", "position_var"),
                     std::move(name));
             },
             "collision_evaluator"_a, "position_var"_a, "name"_a = "DiscreteCollision",
             "Create a discrete collision constraint with one row per contact (rows change on update())")
        .def("getValues", &ti::DiscreteCollisionConstraintD::getValues)
        .def("setBounds", &set_bounds_checked<DiscreteCollisionConstraintDBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::DiscreteCollisionConstraintD::getCollisionEvaluator);

    // ========== ContinuousCollisionConstraintD ==========
    nb::class_<ContinuousCollisionConstraintDBinding, ti::ConstraintSet>(m, "ContinuousCollisionConstraintD")
        .def("__init__", [](ContinuousCollisionConstraintDBinding* self,
                            std::shared_ptr<ti::ContinuousCollisionEvaluator> eval,
                            std::shared_ptr<const ti::Var> var0,
                            std::shared_ptr<const ti::Var> var1,
                            bool fixed0, bool fixed1, const std::string& name) {
                 std::array<std::shared_ptr<const ti::Var>, 2> vars = {
                     require_in_variable_set(var0, "ContinuousCollisionConstraintD", "position_var0"),
                     require_in_variable_set(var1, "ContinuousCollisionConstraintD", "position_var1")};
                 new (self) ContinuousCollisionConstraintDBinding({vars[0], vars[1]}, eval, vars, fixed0, fixed1, name);
             },
             "collision_evaluator"_a, "position_var0"_a, "position_var1"_a,
             "fixed0"_a = false, "fixed1"_a = false, "name"_a = "LVSCollision",
             "Create a continuous collision constraint with one row per contact (rows change on update())")
        .def("getValues", &ti::ContinuousCollisionConstraintD::getValues)
        .def("setBounds", &set_bounds_checked<ContinuousCollisionConstraintDBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::ContinuousCollisionConstraintD::getCollisionEvaluator);

    // ========== ContinuousCollisionNumericalConstraint ==========
    nb::class_<ContinuousCollisionNumericalConstraintBinding, ti::ConstraintSet>(m, "ContinuousCollisionNumericalConstraint")
        .def("__init__", [](ContinuousCollisionNumericalConstraintBinding* self,
                            std::shared_ptr<ti::ContinuousCollisionEvaluator> eval,
                            std::shared_ptr<const ti::Var> var0,
                            std::shared_ptr<const ti::Var> var1,
                            bool fixed0, bool fixed1,
                            int max_num_cnt, bool fixed_sparsity, const std::string& name) {
                 std::array<std::shared_ptr<const ti::Var>, 2> vars = {
                     require_in_variable_set(var0, "ContinuousCollisionNumericalConstraint", "position_var0"),
                     require_in_variable_set(var1, "ContinuousCollisionNumericalConstraint", "position_var1")};
                 new (self) ContinuousCollisionNumericalConstraintBinding({vars[0], vars[1]}, eval, vars, fixed0, fixed1,
                                                                       max_num_cnt, fixed_sparsity, name);
             },
             "collision_evaluator"_a, "position_var0"_a, "position_var1"_a,
             "fixed0"_a = false, "fixed1"_a = false,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "LVSCollision",
             "Create continuous collision constraint with numerical jacobians")
        .def("getValues", &ti::ContinuousCollisionNumericalConstraint::getValues)
        .def("setBounds", &set_bounds_checked<ContinuousCollisionNumericalConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("getCollisionEvaluator", &ti::ContinuousCollisionNumericalConstraint::getCollisionEvaluator);

    // ========== DiscreteCollisionNumericalConstraint ==========
    nb::class_<DiscreteCollisionNumericalConstraintBinding, ti::ConstraintSet>(m, "DiscreteCollisionNumericalConstraint")
        .def("__init__", [](DiscreteCollisionNumericalConstraintBinding* self,
                            std::shared_ptr<ti::DiscreteCollisionEvaluator> eval,
                            const std::shared_ptr<const ti::Var>& var,
                            int max_num_cnt, bool fixed_sparsity, std::string name) {
                 new (self) DiscreteCollisionNumericalConstraintBinding(
                     {var}, std::move(eval),
                     require_in_variable_set(var, "DiscreteCollisionNumericalConstraint", "position_var"),
                     max_num_cnt, fixed_sparsity, std::move(name));
             },
             "collision_evaluator"_a, "position_var"_a,
             "max_num_cnt"_a = 1, "fixed_sparsity"_a = false,
             "name"_a = "DiscreteCollisionNumerical",
             "Create discrete collision constraint with numerical jacobians")
        .def("getValues", &ti::DiscreteCollisionNumericalConstraint::getValues)
        .def("setBounds", &set_bounds_checked<DiscreteCollisionNumericalConstraintBinding>, "bounds"_a,
             "Set the bounds, one per row; raises BoundsSizeError otherwise")
        .def("calcValues", &ti::DiscreteCollisionNumericalConstraint::calcValues, "joint_vals"_a)
        .def("calcJacobianBlock",
             [](const DiscreteCollisionNumericalConstraintBinding& self,
                const Eigen::Ref<const Eigen::VectorXd>& joint_vals) {
                 return jacobian_block_checked(
                     self, joint_vals, self.getCollisionEvaluator()->getJointGroup().numJoints());
             },
             "joint_vals"_a,
             "The constraint Jacobian at joint_vals (sparse matrix, sized like getJacobian()); "
             "needs linked variables and an update() since the last variable change")
        .def("getCollisionEvaluator", &ti::DiscreteCollisionNumericalConstraint::getCollisionEvaluator);

}
