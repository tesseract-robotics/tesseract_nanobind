#include "tesseract_nb.h"

// IFOPT headers
#include <ifopt/bounds.h>
#include <ifopt/composite.h>
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

// Trampoline class for VariableSet (abstract - has pure virtual GetValues/GetBounds/SetVariables)
class PyVariableSet : public ifopt::VariableSet {
public:
    NB_TRAMPOLINE(ifopt::VariableSet, 3);

    ifopt::Component::VectorXd GetValues() const override {
        NB_OVERRIDE_PURE(GetValues);
    }

    ifopt::Component::VecBound GetBounds() const override {
        NB_OVERRIDE_PURE(GetBounds);
    }

    void SetVariables(const ifopt::Component::VectorXd& x) override {
        NB_OVERRIDE_PURE(SetVariables, x);
    }
};

// Trampoline class for ConstraintSet (abstract - has pure virtual GetValues/GetBounds/FillJacobianBlock)
class PyConstraintSet : public ifopt::ConstraintSet {
public:
    NB_TRAMPOLINE(ifopt::ConstraintSet, 3);

    // Expose protected GetVariables() to Python via this public wrapper
    using ifopt::ConstraintSet::GetVariables;

    ifopt::Component::VectorXd GetValues() const override {
        NB_OVERRIDE_PURE(GetValues);
    }

    ifopt::Component::VecBound GetBounds() const override {
        NB_OVERRIDE_PURE(GetBounds);
    }

    void FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& jac_block) const override {
        NB_OVERRIDE_PURE(FillJacobianBlock, var_set, jac_block);
    }
};

// CostTerm: GetCost is private, so we can't create a direct trampoline for it.
// The CostTerm class wraps GetCost into GetValues() which returns a 1-element VectorXd.
// For Python subclassing, users would override GetValues to return their cost.

NB_MODULE(_ifopt, m) {
    m.doc() = "IFOPT optimization library Python bindings (nanobind)";

    // ========== Bounds ==========
    nb::class_<ifopt::Bounds>(m, "Bounds")
        .def(nb::init<>())
        .def(nb::init<double, double>(), "lower"_a, "upper"_a)
        .def_rw("lower", &ifopt::Bounds::lower_)
        .def_rw("upper", &ifopt::Bounds::upper_)
        .def("__iadd__", [](ifopt::Bounds& self, double scalar) {
            self += scalar;
            return self;
        })
        .def("__isub__", [](ifopt::Bounds& self, double scalar) {
            self -= scalar;
            return self;
        })
        .def("__repr__", [](const ifopt::Bounds& self) {
            return "Bounds(" + std::to_string(self.lower_) + ", " + std::to_string(self.upper_) + ")";
        });

    // Bounds constants
    m.attr("inf") = ifopt::inf;
    m.attr("NoBound") = ifopt::NoBound;
    m.attr("BoundZero") = ifopt::BoundZero;
    m.attr("BoundGreaterZero") = ifopt::BoundGreaterZero;
    m.attr("BoundSmallerZero") = ifopt::BoundSmallerZero;

    // ========== Component (abstract base class) ==========
    nb::class_<ifopt::Component>(m, "Component")
        // No init - abstract class
        .def("GetValues", &ifopt::Component::GetValues)
        .def("GetBounds", &ifopt::Component::GetBounds)
        .def("SetVariables", &ifopt::Component::SetVariables)
        .def("GetJacobian", &ifopt::Component::GetJacobian)
        .def("GetRows", &ifopt::Component::GetRows)
        .def("GetName", &ifopt::Component::GetName)
        .def("SetRows", &ifopt::Component::SetRows)
        .def("Print", &ifopt::Component::Print, "tolerance"_a, "index_start"_a)
        .def_prop_ro_static("kSpecifyLater", [](nb::handle) { return ifopt::Component::kSpecifyLater; });

    // ========== Composite ==========
    nb::class_<ifopt::Composite, ifopt::Component>(m, "Composite")
        .def(nb::init<const std::string&, bool>(), "name"_a, "is_cost"_a)
        .def("GetValues", &ifopt::Composite::GetValues)
        .def("GetJacobian", &ifopt::Composite::GetJacobian)
        .def("GetBounds", &ifopt::Composite::GetBounds)
        .def("SetVariables", &ifopt::Composite::SetVariables)
        .def("PrintAll", &ifopt::Composite::PrintAll)
        .def("GetComponent", [](const ifopt::Composite& self, const std::string& name) {
            return self.GetComponent(name);
        })
        .def("AddComponent", &ifopt::Composite::AddComponent)
        .def("ClearComponents", &ifopt::Composite::ClearComponents)
        .def("GetComponents", &ifopt::Composite::GetComponents);

    // ========== VariableSet ==========
    nb::class_<ifopt::VariableSet, ifopt::Component, PyVariableSet>(m, "VariableSet")
        .def(nb::init<int, const std::string&>(), "n_var"_a, "name"_a)
        .def("GetValues", &ifopt::VariableSet::GetValues)
        .def("GetBounds", &ifopt::VariableSet::GetBounds)
        .def("SetVariables", &ifopt::VariableSet::SetVariables);

    // ========== ConstraintSet ==========
    nb::class_<ifopt::ConstraintSet, ifopt::Component, PyConstraintSet>(m, "ConstraintSet")
        .def(nb::init<int, const std::string&>(), "n_constraints"_a, "name"_a)
        .def("GetValues", &ifopt::ConstraintSet::GetValues)
        .def("GetBounds", &ifopt::ConstraintSet::GetBounds)
        .def("GetJacobian", &ifopt::ConstraintSet::GetJacobian)
        .def("GetVariables", &PyConstraintSet::GetVariables)
        .def("LinkWithVariables", &ifopt::ConstraintSet::LinkWithVariables)
        .def("FillJacobianBlock", &ifopt::ConstraintSet::FillJacobianBlock);

    // ========== CostTerm ==========
    // Note: CostTerm inherits from ConstraintSet. GetCost is private but exposed via GetValues()
    // which returns a 1-element VectorXd containing the cost value.
    nb::class_<ifopt::CostTerm, ifopt::ConstraintSet>(m, "CostTerm")
        // No init - abstract class (GetCost is pure virtual)
        .def("GetValues", &ifopt::CostTerm::GetValues)
        .def("GetBounds", &ifopt::CostTerm::GetBounds)
        .def("GetCost", [](const ifopt::CostTerm& self) {
            // GetCost is private, but GetValues returns a 1-element vector with the cost
            return self.GetValues()(0);
        })
        .def("Print", &ifopt::CostTerm::Print, "tolerance"_a, "index"_a);
}
