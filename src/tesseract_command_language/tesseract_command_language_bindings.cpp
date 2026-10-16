/**
 * @file tesseract_command_language_bindings.cpp
 * @brief nanobind bindings for tesseract_command_language
 */

#include "tesseract_nb.h"
#include <algorithm>
#include <limits>
#include <nanobind/stl/map.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/function.h>

// tesseract_command_language - concrete types
#include <tesseract/command_language/joint_waypoint.h>
#include <tesseract/command_language/cartesian_waypoint.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/wait_instruction.h>
#include <tesseract/command_language/timer_instruction.h>
#include <tesseract/command_language/set_analog_instruction.h>
#include <tesseract/command_language/set_digital_instruction.h>
#include <tesseract/command_language/set_tool_instruction.h>
#include <tesseract/command_language/instruction_type.h>
#include <tesseract/common/profile.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/command_language/constants.h>
#include <tesseract/command_language/utils.h>

// tesseract_command_language - Poly types
#include <tesseract/command_language/poly/waypoint_poly.h>
#include <tesseract/command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract/command_language/poly/joint_waypoint_poly.h>
#include <tesseract/command_language/poly/state_waypoint_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/command_language/poly/instruction_poly.h>

// tesseract_common
#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/joint_state.h>

namespace tp = tesseract::command_language;
namespace tc = tesseract::common;

// list.__delitem__ index: negative counts from the end; out of range raises IndexError.
static std::size_t normalize_index(Py_ssize_t i, std::size_t size) {
    const auto n = static_cast<Py_ssize_t>(size);
    const Py_ssize_t j = i < 0 ? i + n : i;
    if (j < 0 || j >= n)
        throw std::out_of_range("index " + std::to_string(i) + " out of range for size " + std::to_string(size));
    return static_cast<std::size_t>(j);
}

// list.insert index: negative counts from the end; out of range clamps to either end.
static std::size_t insert_index(Py_ssize_t i, std::size_t size) {
    const auto n = static_cast<Py_ssize_t>(size);
    const Py_ssize_t j = i < 0 ? i + n : i;
    return static_cast<std::size_t>(std::clamp<Py_ssize_t>(j, 0, n));
}

// CompositeInstruction.insert for one alternative of InstructionPoly (rule poly-alternative-overloads).
template <typename T>
auto composite_insert() {
    return [](tp::CompositeInstruction& self, Py_ssize_t index, const T& instruction) {
        self.insert(self.begin() + static_cast<std::ptrdiff_t>(insert_index(index, self.size())),
                    tp::InstructionPoly(instruction));
    };
}

// UserDataVariant -> Python. AnyPoly is bound in tesseract_task_composer, which this module does not import.
static nb::object user_data_to_python(const std::string& key, const tp::CompositeInstruction::UserDataVariant& value) {
    return std::visit([&key](const auto& v) -> nb::object {
        using V = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<V, std::monostate>)
            return nb::none();
        else if constexpr (std::is_same_v<V, tc::AnyPoly>)
            throw nb::type_error(("CompositeInstruction.getUserData: key '" + key +
                                  "' holds an AnyPoly, which has no Python form in tesseract_command_language")
                                     .c_str());
        else
            return nb::cast(v);
    }, value);
}

// Python -> UserDataVariant. bool before int (bool is an int subclass); an int takes the first of
// int, long, std::size_t that holds it; a float is stored as double.
static tp::CompositeInstruction::UserDataVariant user_data_from_python(const std::string& key, nb::handle value) {
    if (value.is_none()) return std::monostate{};
    if (nb::isinstance<nb::bool_>(value)) return nb::cast<bool>(value);
    if (nb::isinstance<nb::int_>(value)) {
        int i;
        long l;
        std::size_t s;
        if (nb::try_cast(value, i)) return i;
        if (nb::try_cast(value, l)) return l;
        if (nb::try_cast(value, s)) return s;
        throw nb::type_error(("CompositeInstruction.setUserData: int value for key '" + key +
                              "' does not fit int, long or size_t")
                                 .c_str());
    }
    if (nb::isinstance<nb::float_>(value)) return nb::cast<double>(value);
    if (nb::isinstance<nb::str>(value)) return nb::cast<std::string>(value);
    throw nb::type_error(("CompositeInstruction.setUserData: value for key '" + key + "' has unsupported type " +
                          nb::cast<std::string>(nb::type_name(value.type())) +
                          "; expected None, bool, int, float or str")
                             .c_str());
}

// The five UUID members every instruction declares; the UUID is its canonical string (#262).
template <typename Class>
void bind_uuid(Class& cls) {
    using T = typename Class::Type;
    cls.def("getUUID", &T::getUUID)
        .def("setUUID", &T::setUUID, "uuid"_a)
        .def("regenerateUUID", &T::regenerateUUID)
        .def("getParentUUID", &T::getParentUUID)
        .def("setParentUUID", &T::setParentUUID, "uuid"_a);
}

NB_MODULE(_tesseract_command_language, m) {
    m.doc() = "tesseract_command_language Python bindings";

    // ManipulatorInfo is used as a default argument value below — its type
    // must be registered before this module's defs are created.
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");

    // ========== JointWaypoint ==========
    auto joint_waypoint = nb::class_<tp::JointWaypoint>(m, "JointWaypoint")
        .def(nb::init<>())
        .def(nb::init<std::vector<std::string>, const Eigen::VectorXd&, bool>(),
             "names"_a, "position"_a, "is_constrained"_a = true)
        .def(nb::init<std::vector<std::string>, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&>(),
             "names"_a, "position"_a, "lower_tol"_a, "upper_tol"_a)
        .def("setNames", &tp::JointWaypoint::setNames, "names"_a)
        .def("getNames", [](const tp::JointWaypoint& self) { return self.getNames(); })
        .def("setPosition", &tp::JointWaypoint::setPosition, "position"_a)
        .def("getPosition", [](const tp::JointWaypoint& self) -> Eigen::VectorXd { return self.getPosition(); })
        .def("setUpperTolerance", &tp::JointWaypoint::setUpperTolerance, "upper_tol"_a)
        .def("getUpperTolerance", [](const tp::JointWaypoint& self) -> Eigen::VectorXd { return self.getUpperTolerance(); })
        .def("setLowerTolerance", &tp::JointWaypoint::setLowerTolerance, "lower_tol"_a)
        .def("getLowerTolerance", [](const tp::JointWaypoint& self) -> Eigen::VectorXd { return self.getLowerTolerance(); })
        .def("setIsConstrained", &tp::JointWaypoint::setIsConstrained, "value"_a)
        .def("isConstrained", &tp::JointWaypoint::isConstrained)
        .def("setName", &tp::JointWaypoint::setName, "name"_a)
        .def("getName", &tp::JointWaypoint::getName)
        .def("print", &tp::JointWaypoint::print, "prefix"_a = "");
    bind_value_equality(joint_waypoint);
    bind_copy(joint_waypoint);

    // ========== CartesianWaypoint ==========
    auto cartesian_waypoint = nb::class_<tp::CartesianWaypoint>(m, "CartesianWaypoint")
        .def(nb::init<>())
        .def(nb::init<const Eigen::Isometry3d&>(), "transform"_a)
        .def(nb::init<const Eigen::Isometry3d&, const Eigen::VectorXd&, const Eigen::VectorXd&>(),
             "transform"_a, "lower_tol"_a, "upper_tol"_a)
        .def("setTransform", &tp::CartesianWaypoint::setTransform, "transform"_a)
        .def("getTransform", [](const tp::CartesianWaypoint& self) -> Eigen::Isometry3d { return self.getTransform(); })
        .def("setUpperTolerance", &tp::CartesianWaypoint::setUpperTolerance, "upper_tol"_a)
        .def("getUpperTolerance", [](const tp::CartesianWaypoint& self) -> Eigen::VectorXd { return self.getUpperTolerance(); })
        .def("setLowerTolerance", &tp::CartesianWaypoint::setLowerTolerance, "lower_tol"_a)
        .def("getLowerTolerance", [](const tp::CartesianWaypoint& self) -> Eigen::VectorXd { return self.getLowerTolerance(); })
        .def("setSeed", &tp::CartesianWaypoint::setSeed, "seed"_a)
        .def("getSeed", [](const tp::CartesianWaypoint& self) { return self.getSeed(); })
        .def("setName", &tp::CartesianWaypoint::setName, "name"_a)
        .def("getName", &tp::CartesianWaypoint::getName)
        .def("print", &tp::CartesianWaypoint::print, "prefix"_a = "");
    bind_value_equality(cartesian_waypoint);
    bind_copy(cartesian_waypoint);

    // ========== StateWaypoint ==========
    auto state_waypoint = nb::class_<tp::StateWaypoint>(m, "StateWaypoint")
        .def(nb::init<>())
        .def(nb::init<std::vector<std::string>, const Eigen::Ref<const Eigen::VectorXd>&>(),
             "joint_names"_a, "position"_a)
        .def(nb::init<const std::vector<std::string>&, const Eigen::VectorXd&, const Eigen::VectorXd&,
                      const Eigen::VectorXd&, double>(),
             "names"_a, "position"_a, "velocity"_a, "acceleration"_a, "time"_a)
        .def("setNames", &tp::StateWaypoint::setNames, "names"_a)
        .def("getNames", [](const tp::StateWaypoint& self) { return self.getNames(); })
        .def("setPosition", &tp::StateWaypoint::setPosition, "position"_a)
        .def("getPosition", [](const tp::StateWaypoint& self) -> Eigen::VectorXd { return self.getPosition(); })
        .def("setVelocity", &tp::StateWaypoint::setVelocity, "velocity"_a)
        .def("getVelocity", [](const tp::StateWaypoint& self) -> Eigen::VectorXd { return self.getVelocity(); })
        .def("setAcceleration", &tp::StateWaypoint::setAcceleration, "acceleration"_a)
        .def("getAcceleration", [](const tp::StateWaypoint& self) -> Eigen::VectorXd { return self.getAcceleration(); })
        .def("setEffort", &tp::StateWaypoint::setEffort, "effort"_a)
        .def("getEffort", [](const tp::StateWaypoint& self) -> Eigen::VectorXd { return self.getEffort(); })
        .def("setTime", &tp::StateWaypoint::setTime, "time"_a)
        .def("getTime", &tp::StateWaypoint::getTime)
        .def("setName", &tp::StateWaypoint::setName, "name"_a)
        .def("getName", &tp::StateWaypoint::getName)
        .def("print", &tp::StateWaypoint::print, "prefix"_a = "");
    bind_value_equality(state_waypoint);
    bind_copy(state_waypoint);

    // ========== Constants ==========
    m.attr("DEFAULT_PROFILE_KEY") = std::string(tp::DEFAULT_PROFILE_KEY);

    // ========== MoveInstructionType enum ==========
    nb::enum_<tp::MoveInstructionType>(m, "MoveInstructionType")
        .value("LINEAR", tp::MoveInstructionType::LINEAR)
        .value("FREESPACE", tp::MoveInstructionType::FREESPACE)
        .value("CIRCULAR", tp::MoveInstructionType::CIRCULAR);

    // For SWIG compatibility
    m.attr("MoveInstructionType_LINEAR") = tp::MoveInstructionType::LINEAR;
    m.attr("MoveInstructionType_FREESPACE") = tp::MoveInstructionType::FREESPACE;
    m.attr("MoveInstructionType_CIRCULAR") = tp::MoveInstructionType::CIRCULAR;

    // ========== CompositeInstructionOrder enum ==========
    nb::enum_<tp::CompositeInstructionOrder>(m, "CompositeInstructionOrder")
        .value("ORDERED", tp::CompositeInstructionOrder::ORDERED)
        .value("UNORDERED", tp::CompositeInstructionOrder::UNORDERED)
        .value("ORDERED_AND_REVERABLE", tp::CompositeInstructionOrder::ORDERED_AND_REVERABLE);

    // ========== WaypointPoly ==========
    // Typed Polys are declared before WaypointPoly is completed below, so its ctors and casts can name them.
    auto waypoint_poly = nb::class_<tp::WaypointPoly>(m, "WaypointPoly");
    auto cartesian_waypoint_poly = nb::class_<tp::CartesianWaypointPoly>(m, "CartesianWaypointPoly");
    auto joint_waypoint_poly = nb::class_<tp::JointWaypointPoly>(m, "JointWaypointPoly");
    auto state_waypoint_poly = nb::class_<tp::StateWaypointPoly>(m, "StateWaypointPoly");

    // One ctor per alternative: the typed Polys (WaypointInterface) and the concrete waypoints
    // (Cartesian/Joint/StateWaypointInterface).
    waypoint_poly
        .def(nb::init<>())
        .def(nb::init<const tp::CartesianWaypointPoly&>(), "waypoint"_a)
        .def(nb::init<const tp::JointWaypointPoly&>(), "waypoint"_a)
        .def(nb::init<const tp::StateWaypointPoly&>(), "waypoint"_a)
        .def(nb::init<const tp::CartesianWaypoint&>(), "waypoint"_a)
        .def(nb::init<const tp::JointWaypoint&>(), "waypoint"_a)
        .def(nb::init<const tp::StateWaypoint&>(), "waypoint"_a)
        .def("setName", &tp::WaypointPoly::setName, "name"_a)
        .def("getName", &tp::WaypointPoly::getName)
        .def("print", &tp::WaypointPoly::print, "prefix"_a = "")
        .def("isCartesianWaypoint", &tp::WaypointPoly::isCartesianWaypoint)
        .def("isJointWaypoint", &tp::WaypointPoly::isJointWaypoint)
        .def("isStateWaypoint", &tp::WaypointPoly::isStateWaypoint)
        .def("isNull", &tp::WaypointPoly::isNull)
        // C++ as<T>() returns T& and raises std::runtime_error on a type mismatch.
        .def("asCartesianWaypoint", [](tp::WaypointPoly& self) -> tp::CartesianWaypointPoly& {
            return self.as<tp::CartesianWaypointPoly>();
        }, nb::rv_policy::reference_internal, "The stored CartesianWaypointPoly, by reference.")
        .def("asJointWaypoint", [](tp::WaypointPoly& self) -> tp::JointWaypointPoly& {
            return self.as<tp::JointWaypointPoly>();
        }, nb::rv_policy::reference_internal, "The stored JointWaypointPoly, by reference.")
        .def("asStateWaypoint", [](tp::WaypointPoly& self) -> tp::StateWaypointPoly& {
            return self.as<tp::StateWaypointPoly>();
        }, nb::rv_policy::reference_internal, "The stored StateWaypointPoly, by reference.");
    bind_value_equality(waypoint_poly);
    bind_copy(waypoint_poly);

    // ========== CartesianWaypointPoly ==========
    // Note: Not binding as subclass of WaypointPoly due to nanobind limitations
    cartesian_waypoint_poly
        .def(nb::init<>())
        .def(nb::init<tp::CartesianWaypoint>(), "waypoint"_a)
        .def("getTransform", [](const tp::CartesianWaypointPoly& self) -> Eigen::Isometry3d {
            return self.getTransform();
        })
        .def("setTransform", &tp::CartesianWaypointPoly::setTransform, "transform"_a)
        .def("hasSeed", &tp::CartesianWaypointPoly::hasSeed)
        .def("getSeed", [](const tp::CartesianWaypointPoly& self) { return self.getSeed(); })
        .def("clearSeed", &tp::CartesianWaypointPoly::clearSeed)
        .def("setSeed", &tp::CartesianWaypointPoly::setSeed, "seed"_a)
        .def("setUpperTolerance", &tp::CartesianWaypointPoly::setUpperTolerance, "upper_tol"_a)
        .def("getUpperTolerance", [](const tp::CartesianWaypointPoly& self) -> Eigen::VectorXd {
            return self.getUpperTolerance();
        })
        .def("setLowerTolerance", &tp::CartesianWaypointPoly::setLowerTolerance, "lower_tol"_a)
        .def("getLowerTolerance", [](const tp::CartesianWaypointPoly& self) -> Eigen::VectorXd {
            return self.getLowerTolerance();
        })
        .def("isToleranced", &tp::CartesianWaypointPoly::isToleranced)
        .def("asCartesianWaypoint", [](tp::CartesianWaypointPoly& self) -> tp::CartesianWaypoint& {
            return self.as<tp::CartesianWaypoint>();
        }, nb::rv_policy::reference_internal, "The stored CartesianWaypoint, by reference.")
        // Re-expose WaypointPoly methods
        .def("setName", &tp::CartesianWaypointPoly::setName, "name"_a)
        .def("getName", &tp::CartesianWaypointPoly::getName)
        .def("print", &tp::CartesianWaypointPoly::print, "prefix"_a = "")
        .def("isNull", &tp::CartesianWaypointPoly::isNull);
    bind_value_equality(cartesian_waypoint_poly);
    bind_copy(cartesian_waypoint_poly);

    // ========== JointWaypointPoly ==========
    joint_waypoint_poly
        .def(nb::init<>())
        .def(nb::init<tp::JointWaypoint>(), "waypoint"_a)
        .def("getNames", [](const tp::JointWaypointPoly& self) { return self.getNames(); })
        .def("setNames", &tp::JointWaypointPoly::setNames, "names"_a)
        .def("getPosition", [](const tp::JointWaypointPoly& self) -> Eigen::VectorXd {
            return self.getPosition();
        })
        .def("setPosition", &tp::JointWaypointPoly::setPosition, "position"_a)
        .def("isConstrained", &tp::JointWaypointPoly::isConstrained)
        .def("setIsConstrained", &tp::JointWaypointPoly::setIsConstrained, "value"_a)
        .def("setUpperTolerance", &tp::JointWaypointPoly::setUpperTolerance, "upper_tol"_a)
        .def("getUpperTolerance", [](const tp::JointWaypointPoly& self) -> Eigen::VectorXd {
            return self.getUpperTolerance();
        })
        .def("setLowerTolerance", &tp::JointWaypointPoly::setLowerTolerance, "lower_tol"_a)
        .def("getLowerTolerance", [](const tp::JointWaypointPoly& self) -> Eigen::VectorXd {
            return self.getLowerTolerance();
        })
        .def("isToleranced", &tp::JointWaypointPoly::isToleranced)
        .def("asJointWaypoint", [](tp::JointWaypointPoly& self) -> tp::JointWaypoint& {
            return self.as<tp::JointWaypoint>();
        }, nb::rv_policy::reference_internal, "The stored JointWaypoint, by reference.")
        // Re-expose WaypointPoly methods
        .def("setName", &tp::JointWaypointPoly::setName, "name"_a)
        .def("getName", &tp::JointWaypointPoly::getName)
        .def("print", &tp::JointWaypointPoly::print, "prefix"_a = "")
        .def("isNull", &tp::JointWaypointPoly::isNull);
    bind_value_equality(joint_waypoint_poly);
    bind_copy(joint_waypoint_poly);

    // ========== StateWaypointPoly ==========
    state_waypoint_poly
        .def(nb::init<>())
        .def(nb::init<tp::StateWaypoint>(), "waypoint"_a)
        .def("getNames", [](const tp::StateWaypointPoly& self) { return self.getNames(); })
        .def("setNames", &tp::StateWaypointPoly::setNames, "names"_a)
        .def("getPosition", [](const tp::StateWaypointPoly& self) -> Eigen::VectorXd {
            return self.getPosition();
        })
        .def("setPosition", &tp::StateWaypointPoly::setPosition, "position"_a)
        .def("getVelocity", [](const tp::StateWaypointPoly& self) -> Eigen::VectorXd {
            return self.getVelocity();
        })
        .def("setVelocity", &tp::StateWaypointPoly::setVelocity, "velocity"_a)
        .def("getAcceleration", [](const tp::StateWaypointPoly& self) -> Eigen::VectorXd {
            return self.getAcceleration();
        })
        .def("setAcceleration", &tp::StateWaypointPoly::setAcceleration, "acceleration"_a)
        .def("getEffort", [](const tp::StateWaypointPoly& self) -> Eigen::VectorXd {
            return self.getEffort();
        })
        .def("setEffort", &tp::StateWaypointPoly::setEffort, "effort"_a)
        .def("getTime", &tp::StateWaypointPoly::getTime)
        .def("setTime", &tp::StateWaypointPoly::setTime, "time"_a)
        // Re-expose WaypointPoly methods
        .def("setName", &tp::StateWaypointPoly::setName, "name"_a)
        .def("getName", &tp::StateWaypointPoly::getName)
        .def("print", &tp::StateWaypointPoly::print, "prefix"_a = "")
        .def("isNull", &tp::StateWaypointPoly::isNull)
        .def("asStateWaypoint", [](tp::StateWaypointPoly& self) -> tp::StateWaypoint& {
            return self.as<tp::StateWaypoint>();
        }, nb::rv_policy::reference_internal, "The stored StateWaypoint, by reference.");
    bind_value_equality(state_waypoint_poly);
    bind_copy(state_waypoint_poly);

    // ========== Helper functions for Poly wrapping (SWIG compatibility) ==========
    m.def("CartesianWaypointPoly_wrap_CartesianWaypoint", [](const tp::CartesianWaypoint& wp) {
        return tp::CartesianWaypointPoly(wp);
    }, "waypoint"_a);

    m.def("JointWaypointPoly_wrap_JointWaypoint", [](const tp::JointWaypoint& wp) {
        return tp::JointWaypointPoly(wp);
    }, "waypoint"_a);

    m.def("StateWaypointPoly_wrap_StateWaypoint", [](const tp::StateWaypoint& wp) {
        return tp::StateWaypointPoly(wp);
    }, "waypoint"_a);

    // ========== InstructionPoly ==========
    auto instruction_poly = nb::class_<tp::InstructionPoly>(m, "InstructionPoly")
        .def(nb::init<>())
        .def(nb::init<const tp::MoveInstructionPoly&>(), "instruction"_a)
        .def(nb::init<const tp::MoveInstruction&>(), "instruction"_a)
        .def(nb::init<const tp::CompositeInstruction&>(), "instruction"_a)
        .def(nb::init<const tp::WaitInstruction&>(), "instruction"_a)
        .def(nb::init<const tp::TimerInstruction&>(), "instruction"_a)
        .def(nb::init<const tp::SetAnalogInstruction&>(), "instruction"_a)
        .def(nb::init<const tp::SetDigitalInstruction&>(), "instruction"_a)
        .def(nb::init<const tp::SetToolInstruction&>(), "instruction"_a)
        .def("getDescription", &tp::InstructionPoly::getDescription)
        .def("setDescription", &tp::InstructionPoly::setDescription, "description"_a)
        .def("print", &tp::InstructionPoly::print, "prefix"_a = "")
        .def("isCompositeInstruction", &tp::InstructionPoly::isCompositeInstruction)
        .def("isMoveInstruction", &tp::InstructionPoly::isMoveInstruction)
        .def("isWaitInstruction", &tp::isWaitInstruction)
        .def("isTimerInstruction", &tp::isTimerInstruction)
        .def("isSetAnalogInstruction", &tp::isSetAnalogInstruction)
        .def("isSetDigitalInstruction", &tp::isSetDigitalInstruction)
        .def("isSetToolInstruction", &tp::isSetToolInstruction)
        .def("isNull", &tp::InstructionPoly::isNull)
        // Note: In 0.33, use .as<T>() for type casting
        .def("asMoveInstruction", [](tp::InstructionPoly& self) -> tp::MoveInstructionPoly {
            if (!self.isMoveInstruction())
                throw std::runtime_error("InstructionPoly is not a MoveInstruction");
            return self.as<tp::MoveInstructionPoly>();
        }, "Cast to MoveInstructionPoly. Raises RuntimeError if not a move instruction.")
        .def("asCompositeInstruction", [](tp::InstructionPoly& self) -> tp::CompositeInstruction {
            if (!self.isCompositeInstruction())
                throw std::runtime_error("InstructionPoly is not a CompositeInstruction");
            return self.as<tp::CompositeInstruction>();
        }, "Cast to CompositeInstruction. Raises RuntimeError if not a composite instruction.")
        .def("asWaitInstruction", [](tp::InstructionPoly& self) -> tp::WaitInstruction {
            if (!tp::isWaitInstruction(self))
                throw std::runtime_error("InstructionPoly is not a WaitInstruction");
            return self.as<tp::WaitInstruction>();
        }, "Cast to WaitInstruction. Raises RuntimeError if not a wait instruction.")
        .def("asTimerInstruction", [](tp::InstructionPoly& self) -> tp::TimerInstruction {
            if (!tp::isTimerInstruction(self))
                throw std::runtime_error("InstructionPoly is not a TimerInstruction");
            return self.as<tp::TimerInstruction>();
        }, "Cast to TimerInstruction. Raises RuntimeError if not a timer instruction.")
        .def("asSetAnalogInstruction", [](tp::InstructionPoly& self) -> tp::SetAnalogInstruction {
            if (!tp::isSetAnalogInstruction(self))
                throw std::runtime_error("InstructionPoly is not a SetAnalogInstruction");
            return self.as<tp::SetAnalogInstruction>();
        }, "Cast to SetAnalogInstruction. Raises RuntimeError if not a set analog instruction.")
        .def("asSetDigitalInstruction", [](tp::InstructionPoly& self) -> tp::SetDigitalInstruction {
            if (!tp::isSetDigitalInstruction(self))
                throw std::runtime_error("InstructionPoly is not a SetDigitalInstruction");
            return self.as<tp::SetDigitalInstruction>();
        }, "Cast to SetDigitalInstruction. Raises RuntimeError if not a set digital instruction.")
        .def("asSetToolInstruction", [](tp::InstructionPoly& self) -> tp::SetToolInstruction {
            if (!tp::isSetToolInstruction(self))
                throw std::runtime_error("InstructionPoly is not a SetToolInstruction");
            return self.as<tp::SetToolInstruction>();
        }, "Cast to SetToolInstruction. Raises RuntimeError if not a set tool instruction.");
    bind_value_equality(instruction_poly);
    bind_copy(instruction_poly);
    bind_uuid(instruction_poly);

    // ========== MoveInstructionPoly ==========
    // Note: Not binding as subclass due to nanobind cross-module limitations
    auto move_instruction_poly = nb::class_<tp::MoveInstructionPoly>(m, "MoveInstructionPoly")
        .def(nb::init<>())
        .def(nb::init<const tp::MoveInstruction&>(), "instruction"_a)
        .def("getWaypoint", [](tp::MoveInstructionPoly& self) -> tp::WaypointPoly& {
            return self.getWaypoint();
        }, nb::rv_policy::reference_internal)
        // Note: 0.33 removed assignCartesianWaypoint etc. Use setWaypoint instead
        .def("setWaypoint", [](tp::MoveInstructionPoly& self, const tp::CartesianWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a, "Set waypoint from CartesianWaypointPoly")
        .def("setWaypoint", [](tp::MoveInstructionPoly& self, const tp::JointWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a, "Set waypoint from JointWaypointPoly")
        .def("setWaypoint", [](tp::MoveInstructionPoly& self, const tp::StateWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a, "Set waypoint from StateWaypointPoly")
        // SWIG-compatible aliases
        .def("assignCartesianWaypoint", [](tp::MoveInstructionPoly& self, const tp::CartesianWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a)
        .def("assignJointWaypoint", [](tp::MoveInstructionPoly& self, const tp::JointWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a)
        .def("assignStateWaypoint", [](tp::MoveInstructionPoly& self, const tp::StateWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a)
        .def("getMoveType", &tp::MoveInstructionPoly::getMoveType)
        .def("setMoveType", &tp::MoveInstructionPoly::setMoveType, "move_type"_a)
        .def("getProfile", &tp::MoveInstructionPoly::getProfile, "ns"_a = "")
        .def("setProfile", &tp::MoveInstructionPoly::setProfile, "profile"_a)
        .def("getPathProfile", &tp::MoveInstructionPoly::getPathProfile, "ns"_a = "")
        .def("setPathProfile", &tp::MoveInstructionPoly::setPathProfile, "profile"_a)
        .def("setProfileOverrides", &tp::MoveInstructionPoly::setProfileOverrides, "profile_overrides"_a)
        .def("getProfileOverrides", [](const tp::MoveInstructionPoly& self) -> tp::ProfileOverrides {
            return self.getProfileOverrides();
        }, "A copy of the namespace -> profile overrides.")
        .def("setPathProfileOverrides", &tp::MoveInstructionPoly::setPathProfileOverrides, "profile_overrides"_a)
        .def("getPathProfileOverrides", [](const tp::MoveInstructionPoly& self) -> tp::ProfileOverrides {
            return self.getPathProfileOverrides();
        }, "A copy of the namespace -> path profile overrides.")
        .def("createCartesianWaypoint", &tp::MoveInstructionPoly::createCartesianWaypoint)
        .def("createJointWaypoint", &tp::MoveInstructionPoly::createJointWaypoint)
        .def("createStateWaypoint", &tp::MoveInstructionPoly::createStateWaypoint)
        .def("createChild", &tp::MoveInstructionPoly::createChild,
             "A copy whose parent UUID is this instruction's UUID, with a new UUID of its own.")
        .def("isChild", &tp::MoveInstructionPoly::isChild, "True if the parent UUID is not nil.")
        .def("isLinear", &tp::MoveInstructionPoly::isLinear)
        .def("isFreespace", &tp::MoveInstructionPoly::isFreespace)
        .def("isCircular", &tp::MoveInstructionPoly::isCircular)
        .def("getManipulatorInfo", [](const tp::MoveInstructionPoly& self) {
            return self.getManipulatorInfo();
        })
        .def("setManipulatorInfo", &tp::MoveInstructionPoly::setManipulatorInfo, "info"_a)
        // Re-expose InstructionPoly-like methods
        .def("getDescription", &tp::MoveInstructionPoly::getDescription)
        .def("setDescription", &tp::MoveInstructionPoly::setDescription, "description"_a)
        .def("print", &tp::MoveInstructionPoly::print, "prefix"_a = "")
        .def("isNull", &tp::MoveInstructionPoly::isNull)
        .def("asMoveInstruction", [](tp::MoveInstructionPoly& self) -> tp::MoveInstruction& {
            return self.as<tp::MoveInstruction>();
        }, nb::rv_policy::reference_internal, "The stored MoveInstruction, by reference.");
    bind_value_equality(move_instruction_poly);
    bind_copy(move_instruction_poly);
    bind_uuid(move_instruction_poly);

    // Helper for SWIG compatibility
    m.def("MoveInstructionPoly_wrap_MoveInstruction", [](const tp::MoveInstruction& mi) {
        return tp::MoveInstructionPoly(mi);
    }, "instruction"_a);

    // Helper functions to convert Poly types using the new 0.33 .as<T>() API
    m.def("InstructionPoly_as_MoveInstructionPoly", [](tp::InstructionPoly& ip) -> tp::MoveInstructionPoly {
        if (!ip.isMoveInstruction())
            throw std::runtime_error("InstructionPoly is not a MoveInstruction");
        return ip.as<tp::MoveInstructionPoly>();
    }, "instruction"_a);

    m.def("InstructionPoly_as_CompositeInstruction", [](tp::InstructionPoly& ip) -> tp::CompositeInstruction {
        if (!ip.isCompositeInstruction())
            throw std::runtime_error("InstructionPoly is not a CompositeInstruction");
        return ip.as<tp::CompositeInstruction>();
    }, "instruction"_a);

    m.def("InstructionPoly_as_WaitInstruction", [](tp::InstructionPoly& ip) -> tp::WaitInstruction {
        if (!tp::isWaitInstruction(ip))
            throw std::runtime_error("InstructionPoly is not a WaitInstruction");
        return ip.as<tp::WaitInstruction>();
    }, "instruction"_a);

    m.def("InstructionPoly_as_TimerInstruction", [](tp::InstructionPoly& ip) -> tp::TimerInstruction {
        if (!tp::isTimerInstruction(ip))
            throw std::runtime_error("InstructionPoly is not a TimerInstruction");
        return ip.as<tp::TimerInstruction>();
    }, "instruction"_a);

    m.def("InstructionPoly_as_SetAnalogInstruction", [](tp::InstructionPoly& ip) -> tp::SetAnalogInstruction {
        if (!tp::isSetAnalogInstruction(ip))
            throw std::runtime_error("InstructionPoly is not a SetAnalogInstruction");
        return ip.as<tp::SetAnalogInstruction>();
    }, "instruction"_a);

    m.def("InstructionPoly_as_SetDigitalInstruction", [](tp::InstructionPoly& ip) -> tp::SetDigitalInstruction {
        if (!tp::isSetDigitalInstruction(ip))
            throw std::runtime_error("InstructionPoly is not a SetDigitalInstruction");
        return ip.as<tp::SetDigitalInstruction>();
    }, "instruction"_a);

    m.def("InstructionPoly_as_SetToolInstruction", [](tp::InstructionPoly& ip) -> tp::SetToolInstruction {
        if (!tp::isSetToolInstruction(ip))
            throw std::runtime_error("InstructionPoly is not a SetToolInstruction");
        return ip.as<tp::SetToolInstruction>();
    }, "instruction"_a);

    m.def("WaypointPoly_as_StateWaypointPoly", [](tp::WaypointPoly& wp) -> tp::StateWaypointPoly {
        if (!wp.isStateWaypoint())
            throw std::runtime_error("WaypointPoly is not a StateWaypoint");
        return wp.as<tp::StateWaypointPoly>();
    }, "waypoint"_a);

    m.def("WaypointPoly_as_CartesianWaypointPoly", [](tp::WaypointPoly& wp) -> tp::CartesianWaypointPoly {
        if (!wp.isCartesianWaypoint())
            throw std::runtime_error("WaypointPoly is not a CartesianWaypoint");
        return wp.as<tp::CartesianWaypointPoly>();
    }, "waypoint"_a);

    m.def("WaypointPoly_as_JointWaypointPoly", [](tp::WaypointPoly& wp) -> tp::JointWaypointPoly {
        if (!wp.isJointWaypoint())
            throw std::runtime_error("WaypointPoly is not a JointWaypoint");
        return wp.as<tp::JointWaypointPoly>();
    }, "waypoint"_a);

    // ========== MoveInstruction ==========
    // The C++ ctors take a WaypointPoly; the per-type overloads name what callers pass
    // (rule poly-alternative-overloads). Without a path profile, C++ sets it to `profile`
    // for LINEAR/CIRCULAR moves and leaves it empty for FREESPACE.
    auto move_instruction = nb::class_<tp::MoveInstruction>(m, "MoveInstruction")
        .def(nb::init<tp::WaypointPoly, tp::MoveInstructionType, std::string, tc::ManipulatorInfo>(),
             "waypoint"_a, "type"_a, "profile"_a = std::string(tp::DEFAULT_PROFILE_KEY),
             "manipulator_info"_a = tc::ManipulatorInfo())
        .def(nb::init<tp::CartesianWaypointPoly, tp::MoveInstructionType, std::string, tc::ManipulatorInfo>(),
             "waypoint"_a, "type"_a, "profile"_a = std::string(tp::DEFAULT_PROFILE_KEY),
             "manipulator_info"_a = tc::ManipulatorInfo())
        .def(nb::init<tp::JointWaypointPoly, tp::MoveInstructionType, std::string, tc::ManipulatorInfo>(),
             "waypoint"_a, "type"_a, "profile"_a = std::string(tp::DEFAULT_PROFILE_KEY),
             "manipulator_info"_a = tc::ManipulatorInfo())
        .def(nb::init<tp::StateWaypointPoly, tp::MoveInstructionType, std::string, tc::ManipulatorInfo>(),
             "waypoint"_a, "type"_a, "profile"_a = std::string(tp::DEFAULT_PROFILE_KEY),
             "manipulator_info"_a = tc::ManipulatorInfo())
        .def(nb::init<tp::WaypointPoly, tp::MoveInstructionType, std::string, std::string, tc::ManipulatorInfo>(),
             "waypoint"_a, "type"_a, "profile"_a, "path_profile"_a, "manipulator_info"_a = tc::ManipulatorInfo())
        .def(nb::init<tp::CartesianWaypointPoly, tp::MoveInstructionType, std::string, std::string,
                      tc::ManipulatorInfo>(),
             "waypoint"_a, "type"_a, "profile"_a, "path_profile"_a, "manipulator_info"_a = tc::ManipulatorInfo())
        .def(nb::init<tp::JointWaypointPoly, tp::MoveInstructionType, std::string, std::string,
                      tc::ManipulatorInfo>(),
             "waypoint"_a, "type"_a, "profile"_a, "path_profile"_a, "manipulator_info"_a = tc::ManipulatorInfo())
        .def(nb::init<tp::StateWaypointPoly, tp::MoveInstructionType, std::string, std::string,
                      tc::ManipulatorInfo>(),
             "waypoint"_a, "type"_a, "profile"_a, "path_profile"_a, "manipulator_info"_a = tc::ManipulatorInfo())
        .def("getWaypoint", [](tp::MoveInstruction& self) -> tp::WaypointPoly& {
            return self.getWaypoint();
        }, nb::rv_policy::reference_internal)
        // Note: 0.33 removed assign*Waypoint methods - use setWaypoint or direct getWaypoint() assignment
        .def("setWaypoint", [](tp::MoveInstruction& self, const tp::CartesianWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a)
        .def("setWaypoint", [](tp::MoveInstruction& self, const tp::JointWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a)
        .def("setWaypoint", [](tp::MoveInstruction& self, const tp::StateWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a)
        // SWIG-compatible aliases
        .def("assignCartesianWaypoint", [](tp::MoveInstruction& self, const tp::CartesianWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a)
        .def("assignJointWaypoint", [](tp::MoveInstruction& self, const tp::JointWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a)
        .def("assignStateWaypoint", [](tp::MoveInstruction& self, const tp::StateWaypointPoly& wp) {
            self.getWaypoint() = wp;
        }, "waypoint"_a)
        .def("getMoveType", &tp::MoveInstruction::getMoveType)
        .def("setMoveType", &tp::MoveInstruction::setMoveType, "move_type"_a)
        .def("getProfile", &tp::MoveInstruction::getProfile, "ns"_a = "")
        .def("setProfile", &tp::MoveInstruction::setProfile, "profile"_a)
        .def("getPathProfile", &tp::MoveInstruction::getPathProfile, "ns"_a = "")
        .def("setPathProfile", &tp::MoveInstruction::setPathProfile, "profile"_a)
        .def("setProfileOverrides", &tp::MoveInstruction::setProfileOverrides, "profile_overrides"_a)
        .def("getProfileOverrides", [](const tp::MoveInstruction& self) -> tp::ProfileOverrides {
            return self.getProfileOverrides();
        }, "A copy of the namespace -> profile overrides.")
        .def("setPathProfileOverrides", &tp::MoveInstruction::setPathProfileOverrides, "profile_overrides"_a)
        .def("getPathProfileOverrides", [](const tp::MoveInstruction& self) -> tp::ProfileOverrides {
            return self.getPathProfileOverrides();
        }, "A copy of the namespace -> path profile overrides.")
        .def("createCartesianWaypoint", &tp::MoveInstruction::createCartesianWaypoint)
        .def("createJointWaypoint", &tp::MoveInstruction::createJointWaypoint)
        .def("createStateWaypoint", &tp::MoveInstruction::createStateWaypoint)
        .def("getManipulatorInfo", [](const tp::MoveInstruction& self) {
            return self.getManipulatorInfo();
        })
        .def("setManipulatorInfo", &tp::MoveInstruction::setManipulatorInfo, "info"_a)
        .def("print", &tp::MoveInstruction::print, "prefix"_a = "")
        .def("getDescription", &tp::MoveInstruction::getDescription)
        .def("setDescription", &tp::MoveInstruction::setDescription, "description"_a);
    bind_value_equality(move_instruction);
    bind_copy(move_instruction);
    bind_uuid(move_instruction);

    // ========== WaitInstructionType ==========
    nb::enum_<tp::WaitInstructionType>(m, "WaitInstructionType")
        .value("TIME", tp::WaitInstructionType::TIME)
        .value("DIGITAL_INPUT_HIGH", tp::WaitInstructionType::DIGITAL_INPUT_HIGH)
        .value("DIGITAL_INPUT_LOW", tp::WaitInstructionType::DIGITAL_INPUT_LOW)
        .value("DIGITAL_OUTPUT_HIGH", tp::WaitInstructionType::DIGITAL_OUTPUT_HIGH)
        .value("DIGITAL_OUTPUT_LOW", tp::WaitInstructionType::DIGITAL_OUTPUT_LOW);

    // ========== WaitInstruction ==========
    auto wait_instruction = nb::class_<tp::WaitInstruction>(m, "WaitInstruction")
        .def(nb::init<double>(), "time"_a, "Construct a TIME-mode wait of `time` seconds.")
        .def(nb::init<tp::WaitInstructionType, int>(), "type"_a, "io"_a,
             "Construct a digital-IO wait. `type` is one of the DIGITAL_* modes; `io` is the IO index.")
        .def("getWaitType", &tp::WaitInstruction::getWaitType)
        .def("setWaitType", &tp::WaitInstruction::setWaitType, "type"_a)
        .def("getWaitTime", &tp::WaitInstruction::getWaitTime)
        .def("setWaitTime", &tp::WaitInstruction::setWaitTime, "time"_a)
        .def("getWaitIO", &tp::WaitInstruction::getWaitIO)
        .def("setWaitIO", &tp::WaitInstruction::setWaitIO, "io"_a)
        .def("getDescription", &tp::WaitInstruction::getDescription)
        .def("setDescription", &tp::WaitInstruction::setDescription, "description"_a)
        .def("print", &tp::WaitInstruction::print, "prefix"_a = "");
    bind_value_equality(wait_instruction);
    bind_copy(wait_instruction);
    bind_uuid(wait_instruction);

    // ========== TimerInstructionType ==========
    nb::enum_<tp::TimerInstructionType>(m, "TimerInstructionType")
        .value("DIGITAL_OUTPUT_HIGH", tp::TimerInstructionType::DIGITAL_OUTPUT_HIGH)
        .value("DIGITAL_OUTPUT_LOW", tp::TimerInstructionType::DIGITAL_OUTPUT_LOW);

    // ========== TimerInstruction ==========
    auto timer_instruction = nb::class_<tp::TimerInstruction>(m, "TimerInstruction")
        .def(nb::init<tp::TimerInstructionType, double, int>(), "type"_a, "time"_a, "io"_a,
             "Construct a timer that, after `time` seconds, drives digital output `io` HIGH or LOW per `type`.")
        .def("getTimerType", &tp::TimerInstruction::getTimerType)
        .def("setTimerType", &tp::TimerInstruction::setTimerType, "type"_a)
        .def("getTimerTime", &tp::TimerInstruction::getTimerTime)
        .def("setTimerTime", &tp::TimerInstruction::setTimerTime, "time"_a)
        .def("getTimerIO", &tp::TimerInstruction::getTimerIO)
        .def("setTimerIO", &tp::TimerInstruction::setTimerIO, "io"_a)
        .def("getDescription", &tp::TimerInstruction::getDescription)
        .def("setDescription", &tp::TimerInstruction::setDescription, "description"_a)
        .def("print", &tp::TimerInstruction::print, "prefix"_a = "");
    bind_value_equality(timer_instruction);
    bind_copy(timer_instruction);
    bind_uuid(timer_instruction);

    // ========== SetAnalogInstruction ==========
    auto set_analog_instruction = nb::class_<tp::SetAnalogInstruction>(m, "SetAnalogInstruction")
        .def(nb::init<std::string, int, double>(), "key"_a, "index"_a, "value"_a,
             "Set analog channel identified by (key, index) to `value`.")
        .def("getKey", &tp::SetAnalogInstruction::getKey)
        .def("getIndex", &tp::SetAnalogInstruction::getIndex)
        .def("getValue", &tp::SetAnalogInstruction::getValue)
        .def("getDescription", &tp::SetAnalogInstruction::getDescription)
        .def("setDescription", &tp::SetAnalogInstruction::setDescription, "description"_a)
        .def("print", &tp::SetAnalogInstruction::print, "prefix"_a = "");
    bind_value_equality(set_analog_instruction);
    bind_copy(set_analog_instruction);
    bind_uuid(set_analog_instruction);

    // ========== SetDigitalInstruction ==========
    auto set_digital_instruction = nb::class_<tp::SetDigitalInstruction>(m, "SetDigitalInstruction")
        .def(nb::init<std::string, int, bool>(), "key"_a, "index"_a, "value"_a,
             "Set digital channel identified by (key, index) to `value`.")
        .def("getKey", &tp::SetDigitalInstruction::getKey)
        .def("getIndex", &tp::SetDigitalInstruction::getIndex)
        .def("getValue", &tp::SetDigitalInstruction::getValue)
        .def("getDescription", &tp::SetDigitalInstruction::getDescription)
        .def("setDescription", &tp::SetDigitalInstruction::setDescription, "description"_a)
        .def("print", &tp::SetDigitalInstruction::print, "prefix"_a = "");
    bind_value_equality(set_digital_instruction);
    bind_copy(set_digital_instruction);
    bind_uuid(set_digital_instruction);

    // ========== SetToolInstruction ==========
    auto set_tool_instruction = nb::class_<tp::SetToolInstruction>(m, "SetToolInstruction")
        .def(nb::init<int>(), "tool_id"_a, "Activate the tool with the given ID.")
        .def("getTool", &tp::SetToolInstruction::getTool)
        .def("getDescription", &tp::SetToolInstruction::getDescription)
        .def("setDescription", &tp::SetToolInstruction::setDescription, "description"_a)
        .def("print", &tp::SetToolInstruction::print, "prefix"_a = "");
    bind_value_equality(set_tool_instruction);
    bind_copy(set_tool_instruction);
    bind_uuid(set_tool_instruction);

    // ========== CompositeInstruction ==========
    auto composite_instruction = nb::class_<tp::CompositeInstruction>(m, "CompositeInstruction")
        .def(nb::init<>())
        .def(nb::init<std::string>(), "profile"_a)  // SWIG-compatible constructor
        // Full C++ ctor — e.g. raster transition composites need order=UNORDERED
        .def(nb::init<std::string, tesseract::common::ManipulatorInfo, tp::CompositeInstructionOrder>(),
             "profile"_a, "manipulator_info"_a = tesseract::common::ManipulatorInfo(),
             "order"_a = tp::CompositeInstructionOrder::ORDERED)
        .def("getOrder", &tp::CompositeInstruction::getOrder)
        .def("getDescription", &tp::CompositeInstruction::getDescription)
        .def("setDescription", &tp::CompositeInstruction::setDescription, "description"_a)
        .def("print", &tp::CompositeInstruction::print, "prefix"_a = "")
        .def("getProfile", &tp::CompositeInstruction::getProfile, "ns"_a = "")
        .def("setProfile", &tp::CompositeInstruction::setProfile, "profile"_a)
        .def("setProfileOverrides", &tp::CompositeInstruction::setProfileOverrides, "profile_overrides"_a)
        .def("getProfileOverrides", [](const tp::CompositeInstruction& self) -> tp::ProfileOverrides {
            return self.getProfileOverrides();
        }, "A copy of the namespace -> profile overrides.")
        .def("getUserData", [](const tp::CompositeInstruction& self) {
            nb::dict out;
            for (const auto& [key, value] : self.getUserData())
                out[nb::cast(key)] = user_data_to_python(key, value);
            return out;
        }, nb::sig("def getUserData(self) -> dict[str, None | bool | int | float | str]"),
           "A copy of the user data.\n\n"
           "C++ int, long and size_t values all read back as int; float and double as float. An AnyPoly "
           "value raises TypeError: AnyPoly is bound in tesseract_task_composer.")
        .def("setUserData", [](tp::CompositeInstruction& self, const nb::dict& data) {
            tp::CompositeInstruction::UserData user_data;
            for (const auto& [key, value] : data) {
                if (!nb::isinstance<nb::str>(key))
                    throw nb::type_error("CompositeInstruction.setUserData: keys must be str");
                const auto k = nb::cast<std::string>(key);
                user_data[k] = user_data_from_python(k, value);
            }
            self.getUserData() = std::move(user_data);
        }, "data"_a, nb::sig("def setUserData(self, data: dict[str, None | bool | int | float | str]) -> None"),
           "Replace the user data.\n\n"
           "None is stored as std::monostate, bool as bool, float as double, str as std::string. An int is "
           "stored as the first of int, long and size_t that holds it, so a C++ size_t written as a small "
           "value reads back as int. Any other value raises TypeError and leaves the user data unchanged.")
        .def("getManipulatorInfo", [](const tp::CompositeInstruction& self) {
            return self.getManipulatorInfo();
        })
        .def("setManipulatorInfo", &tp::CompositeInstruction::setManipulatorInfo, "info"_a)
        .def("getInstructions", [](tp::CompositeInstruction& self) -> std::vector<tp::InstructionPoly>& {
            return self.getInstructions();
        }, nb::rv_policy::reference_internal)
        .def("setInstructions", &tp::CompositeInstruction::setInstructions, "instructions"_a)
        // Per-type push_back overloads exist primarily for static type-checkers:
        // nb::implicitly_convertible<T, InstructionPoly> handles dispatch at runtime,
        // but stub generators emit each .def with its declared signature, so without
        // explicit overloads, a callsite like `composite.push_back(MoveInstructionPoly(...))`
        // would type-check as an error even though it works at runtime.
        .def("push_back", [](tp::CompositeInstruction& self, const tp::InstructionPoly& ip) {
            self.push_back(ip);
        }, "instruction"_a)
        .def("push_back", [](tp::CompositeInstruction& self, const tp::MoveInstructionPoly& mi) {
            self.push_back(tp::InstructionPoly(mi));
        }, "instruction"_a)
        .def("push_back", [](tp::CompositeInstruction& self, const tp::MoveInstruction& mi) {
            self.push_back(tp::InstructionPoly(mi));
        }, "instruction"_a)
        .def("push_back", [](tp::CompositeInstruction& self, const tp::CompositeInstruction& ci) {
            self.push_back(tp::InstructionPoly(ci));
        }, "instruction"_a)
        .def("push_back", [](tp::CompositeInstruction& self, const tp::WaitInstruction& wi) {
            self.push_back(tp::InstructionPoly(wi));
        }, "instruction"_a)
        .def("push_back", [](tp::CompositeInstruction& self, const tp::TimerInstruction& ti) {
            self.push_back(tp::InstructionPoly(ti));
        }, "instruction"_a)
        .def("push_back", [](tp::CompositeInstruction& self, const tp::SetAnalogInstruction& si) {
            self.push_back(tp::InstructionPoly(si));
        }, "instruction"_a)
        .def("push_back", [](tp::CompositeInstruction& self, const tp::SetDigitalInstruction& si) {
            self.push_back(tp::InstructionPoly(si));
        }, "instruction"_a)
        .def("push_back", [](tp::CompositeInstruction& self, const tp::SetToolInstruction& si) {
            self.push_back(tp::InstructionPoly(si));
        }, "instruction"_a)
        // SWIG-compatible alias
        .def("appendMoveInstruction", [](tp::CompositeInstruction& self, const tp::MoveInstructionPoly& mi) {
            self.push_back(mi);
        }, "mi"_a)
        // list.insert form: the index becomes an iterator. One overload per alternative, as push_back.
        .def("insert", composite_insert<tp::InstructionPoly>(), "index"_a, "instruction"_a)
        .def("insert", composite_insert<tp::MoveInstructionPoly>(), "index"_a, "instruction"_a)
        .def("insert", composite_insert<tp::MoveInstruction>(), "index"_a, "instruction"_a)
        .def("insert", composite_insert<tp::CompositeInstruction>(), "index"_a, "instruction"_a)
        .def("insert", composite_insert<tp::WaitInstruction>(), "index"_a, "instruction"_a)
        .def("insert", composite_insert<tp::TimerInstruction>(), "index"_a, "instruction"_a)
        .def("insert", composite_insert<tp::SetAnalogInstruction>(), "index"_a, "instruction"_a)
        .def("insert", composite_insert<tp::SetDigitalInstruction>(), "index"_a, "instruction"_a)
        .def("insert", composite_insert<tp::SetToolInstruction>(), "index"_a, "instruction"_a)
        // erase, in its Python form.
        .def("__delitem__", [](tp::CompositeInstruction& self, Py_ssize_t i) {
            self.erase(self.begin() + static_cast<std::ptrdiff_t>(normalize_index(i, self.size())));
        }, "index"_a)
        .def("__delitem__", [](tp::CompositeInstruction& self, const nb::slice& slice) {
            const auto [start, stop, step, length] = slice.compute(self.size());
            if (step == 1) {
                self.erase(self.begin() + start, self.begin() + start + static_cast<Py_ssize_t>(length));
                return;
            }
            // Erase from the back, so the indices still to erase stay valid.
            std::vector<Py_ssize_t> indices;
            for (std::size_t k = 0; k < length; ++k)
                indices.push_back(start + static_cast<Py_ssize_t>(k) * step);
            std::sort(indices.rbegin(), indices.rend());
            for (const Py_ssize_t j : indices)
                self.erase(self.begin() + j);
        }, "slice"_a)
        .def("size", &tp::CompositeInstruction::size)
        .def("empty", &tp::CompositeInstruction::empty)
        .def("clear", &tp::CompositeInstruction::clear)
        .def("max_size", &tp::CompositeInstruction::max_size)
        .def("reserve", &tp::CompositeInstruction::reserve, "n"_a)
        .def("capacity", &tp::CompositeInstruction::capacity)
        .def("shrink_to_fit", &tp::CompositeInstruction::shrink_to_fit)
        // front/back/pop_back on an empty std::vector are undefined behaviour: check first.
        .def("front", [](tp::CompositeInstruction& self) -> tp::InstructionPoly& {
            if (self.empty()) throw std::out_of_range("CompositeInstruction.front: empty composite");
            return self.front();
        }, nb::rv_policy::reference_internal)
        .def("back", [](tp::CompositeInstruction& self) -> tp::InstructionPoly& {
            if (self.empty()) throw std::out_of_range("CompositeInstruction.back: empty composite");
            return self.back();
        }, nb::rv_policy::reference_internal)
        .def("at", nb::overload_cast<tp::CompositeInstruction::size_type>(&tp::CompositeInstruction::at),
             "n"_a, nb::rv_policy::reference_internal)
        .def("pop_back", [](tp::CompositeInstruction& self) {
            if (self.empty()) throw std::out_of_range("CompositeInstruction.pop_back: empty composite");
            self.pop_back();
        })
        // Traversal into child composites. Instructions come back by reference into this
        // composite, which they keep alive. A filter is called as filter(instruction, parent).
        .def("flatten", [](tp::CompositeInstruction& self, const tp::flattenFilterFn& filter) {
            const nb::handle parent = nb::find(&self);
            nb::typed<nb::list, tp::InstructionPoly> out;
            for (const auto& ref : self.flatten(filter))
                out.append(nb::cast(&ref.get(), nb::rv_policy::reference_internal, parent));
            return out;
        }, "filter"_a = nb::none(),
           "Every leaf instruction, descending into child composites, that `filter` accepts (all if None).")
        .def("getFirstMoveInstruction",
             nb::overload_cast<>(&tp::CompositeInstruction::getFirstMoveInstruction),
             nb::rv_policy::reference_internal, "The first move instruction, descending into children, or None.")
        .def("getLastMoveInstruction",
             nb::overload_cast<>(&tp::CompositeInstruction::getLastMoveInstruction),
             nb::rv_policy::reference_internal, "The last move instruction, descending into children, or None.")
        .def("getMoveInstructionCount", &tp::CompositeInstruction::getMoveInstructionCount)
        .def("getFirstInstruction",
             nb::overload_cast<const tp::locateFilterFn&, bool>(&tp::CompositeInstruction::getFirstInstruction),
             "locate_filter"_a = nb::none(), "process_child_composites"_a = true,
             nb::rv_policy::reference_internal, "The first instruction `locate_filter` accepts, or None.")
        .def("getLastInstruction",
             nb::overload_cast<const tp::locateFilterFn&, bool>(&tp::CompositeInstruction::getLastInstruction),
             "locate_filter"_a = nb::none(), "process_child_composites"_a = true,
             nb::rv_policy::reference_internal, "The last instruction `locate_filter` accepts, or None.")
        .def("getInstructionCount", &tp::CompositeInstruction::getInstructionCount,
             "locate_filter"_a = nb::none(), "process_child_composites"_a = true)
        .def("__len__", &tp::CompositeInstruction::size)
        .def("__getitem__", [](tp::CompositeInstruction& self, std::size_t i) -> tp::InstructionPoly& {
            if (i >= self.size()) throw std::out_of_range("Index out of range");
            return self[i];
        }, nb::rv_policy::reference_internal)
        .def("__iter__", [](tp::CompositeInstruction& self) {
            return nb::make_iterator(nb::type<tp::CompositeInstruction>(), "iterator", self.begin(), self.end());
        }, nb::keep_alive<0, 1>());
    bind_value_equality(composite_instruction);
    bind_copy(composite_instruction);
    bind_uuid(composite_instruction);

    m.def("moveFilter", &tp::moveFilter, "instruction"_a, "composite"_a,
          "Filter for flatten/getFirstInstruction/...: True for a move instruction.");

    // ========== Profile (base class) ==========
    // Note: Profile moved to tesseract_common in 0.33
    nb::class_<tc::Profile>(m, "Profile")
        .def(nb::init<>())
        .def(nb::init<std::size_t>(), "key"_a)
        .def("getKey", &tc::Profile::getKey);

    // ========== ProfileDictionary ==========
    // Note: ProfileDictionary moved to tesseract_common in 0.33
    // Note: key is now std::size_t (hash), use Profile::createKey<T>() to get it
    nb::class_<tc::ProfileDictionary>(m, "ProfileDictionary")
        .def(nb::init<>())
        .def("addProfile", nb::overload_cast<const std::string&, const std::string&, const tc::Profile::ConstPtr&>(
            &tc::ProfileDictionary::addProfile), "ns"_a, "profile_name"_a, "profile"_a)
        .def("addProfile", nb::overload_cast<const std::string&, const std::vector<std::string>&,
                                             const tc::Profile::ConstPtr&>(&tc::ProfileDictionary::addProfile),
             "ns"_a, "profile_names"_a, "profile"_a, "Add one profile under each of `profile_names`.")
        .def("hasProfile", &tc::ProfileDictionary::hasProfile, "key"_a, "ns"_a, "profile_name"_a)
        .def("getProfile", [](const tc::ProfileDictionary& self, std::size_t key, const std::string& ns, const std::string& profile_name) {
            return self.getProfile(key, ns, profile_name);
        }, "key"_a, "ns"_a, "profile_name"_a)
        .def("removeProfile", &tc::ProfileDictionary::removeProfile, "key"_a, "ns"_a, "profile_name"_a)
        .def("hasProfileEntry", &tc::ProfileDictionary::hasProfileEntry, "key"_a, "ns"_a)
        .def("removeProfileEntry", &tc::ProfileDictionary::removeProfileEntry, "key"_a, "ns"_a)
        .def("getProfileEntry", &tc::ProfileDictionary::getProfileEntry, "key"_a, "ns"_a,
             "Profile name -> profile for `key` in `ns`. A copy of the map; the profiles are shared.")
        .def("getAllProfileEntries", &tc::ProfileDictionary::getAllProfileEntries,
             "ns -> key -> profile name -> profile. A copy of the maps; the profiles are shared.")
        .def("clear", &tc::ProfileDictionary::clear);

    // Helper function to add profiles from other modules (cross-module inheritance workaround)
    // This casts Profile::ConstPtr from other modules to the base type
    m.def("ProfileDictionary_addProfile", [](tc::ProfileDictionary& dict,
                                              const std::string& ns,
                                              const std::string& profile_name,
                                              tc::Profile::ConstPtr profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add a profile to the dictionary (cross-module helper)");

    // ========== utils.h ==========
    // WaypointPoly forms only: upstream deletes the template siblings so a typed waypoint is never
    // converted to a temporary WaypointPoly. No WaypointPoly implicit conversion is registered, so
    // the in-place functions edit the object Python passes (e.g. move.getWaypoint()).
    m.def("toJointTrajectory", nb::overload_cast<const tp::InstructionPoly&>(&tp::toJointTrajectory),
          "instruction"_a, "The joint states of every move instruction, descending into child composites.");
    m.def("toJointTrajectory", nb::overload_cast<const tp::CompositeInstruction&>(&tp::toJointTrajectory),
          "composite_instructions"_a,
          "The joint states of every move instruction, descending into child composites.");
    m.def("getJointPosition", [](const tp::WaypointPoly& waypoint) -> Eigen::VectorXd {
        return tp::getJointPosition(waypoint);
    }, "waypoint"_a, "The joint position of a joint or state waypoint; raises for a Cartesian waypoint.");
    m.def("getJointPosition", [](const std::vector<std::string>& joint_names, const tp::WaypointPoly& waypoint) {
        return tp::getJointPosition(joint_names, waypoint);
    }, "joint_names"_a, "waypoint"_a, "The joint position ordered by `joint_names`.");
    m.def("getJointNames", [](const tp::WaypointPoly& waypoint) { return tp::getJointNames(waypoint); },
          "waypoint"_a, "The joint names of a joint or state waypoint; raises for a Cartesian waypoint.");
    m.def("formatJointPosition", [](const std::vector<std::string>& joint_names, tp::WaypointPoly& waypoint) {
        return tp::formatJointPosition(joint_names, waypoint);
    }, "joint_names"_a, "waypoint"_a, "Reorder `waypoint` in place to `joint_names`; True if it was reordered.");
    m.def("checkJointPositionFormat", &tp::checkJointPositionFormat, "joint_names"_a, "waypoint"_a,
          "True if the waypoint's joints are ordered as `joint_names`.");
    m.def("setJointPosition", [](tp::WaypointPoly& waypoint, const Eigen::Ref<const Eigen::VectorXd>& position) {
        return tp::setJointPosition(waypoint, position);
    }, "waypoint"_a, "position"_a, "Set the joint position in place; False for a Cartesian waypoint.");
    m.def("isWithinJointLimits", [](const tp::WaypointPoly& waypoint, const Eigen::Ref<const Eigen::MatrixX2d>& limits) {
        return tp::isWithinJointLimits(waypoint, limits);
    }, "waypoint"_a, "limits"_a, "`limits` is (n, 2): lower, upper. True for a Cartesian waypoint.");
    // The scalar default is the C++ one, std::numeric_limits<double>::max(), not float("inf").
    m.def("clampToJointLimits", [](tp::WaypointPoly& waypoint, const Eigen::Ref<const Eigen::MatrixX2d>& limits,
                                   double max_deviation) {
        return tp::clampToJointLimits(waypoint, limits, max_deviation);
    }, "waypoint"_a, "limits"_a, "max_deviation"_a = std::numeric_limits<double>::max(),
          "Clamp in place to `limits` ((n, 2): lower, upper) when every joint is within `max_deviation` of "
          "its limit; False otherwise.");
    m.def("clampToJointLimits", [](tp::WaypointPoly& waypoint, const Eigen::Ref<const Eigen::MatrixX2d>& limits,
                                   const Eigen::Ref<const Eigen::VectorXd>& max_deviation) {
        return tp::clampToJointLimits(waypoint, limits, max_deviation);
    }, "waypoint"_a, "limits"_a, "max_deviation"_a, "As above, with one maximum deviation per joint.");
    m.def("toDelimitedFile", [](const tp::CompositeInstruction& composite_instructions,
                                const std::filesystem::path& file_path, char separator) {
        return tp::toDelimitedFile(composite_instructions, file_path.string(), separator);
    }, "composite_instructions"_a, "file_path"_a, "separator"_a = ',',
          "Write the joint positions of every move instruction to `file_path`; True on success.");
    // validateSeedStructure: declared in utils.h, defined nowhere upstream (0.35.0), so not bound.
    m.def("makeTimeMonotonicallyIncreasing", &tp::makeTimeMonotonicallyIncreasing, "composite_instructions"_a,
          "Make the state-waypoint times strictly increasing across the whole program, in place.");

    // Python-side convertibility: lets any API accepting an InstructionPoly
    // (push_back, setInstructions, function parameters) be called with a
    // MoveInstructionPoly or a CompositeInstruction directly. Pairs with
    // the InstructionPoly(nb::init<const SubType&>()) constructors bound
    // above. Must come after all three class bindings.
    nb::implicitly_convertible<tp::MoveInstructionPoly, tp::InstructionPoly>();
    nb::implicitly_convertible<tp::CompositeInstruction, tp::InstructionPoly>();
    nb::implicitly_convertible<tp::MoveInstructionPoly, tp::InstructionPoly>();
    nb::implicitly_convertible<tp::CompositeInstruction, tp::InstructionPoly>();
    nb::implicitly_convertible<tp::WaitInstruction, tp::InstructionPoly>();
    nb::implicitly_convertible<tp::TimerInstruction, tp::InstructionPoly>();
    nb::implicitly_convertible<tp::SetAnalogInstruction, tp::InstructionPoly>();
    nb::implicitly_convertible<tp::SetDigitalInstruction, tp::InstructionPoly>();
    nb::implicitly_convertible<tp::SetToolInstruction, tp::InstructionPoly>();
    nb::implicitly_convertible<tp::MoveInstruction, tp::InstructionPoly>();

}
