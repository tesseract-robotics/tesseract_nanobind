/**
 * @file ompl_base_bindings.cpp
 * @brief nanobind bindings for OMPL base state spaces (SE2, Reeds-Shepp, Dubins)
 *
 * Exposes car-like vehicle planning primitives from OMPL.
 */

#include "tesseract_nb.h"

#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

namespace ob = ompl::base;

NB_MODULE(_ompl_base, m) {
    m.doc() = "OMPL base state space bindings for car-like vehicle planning";

    // ========== State (opaque handle) ==========
    // State is an abstract base class - we expose it as an opaque pointer
    // that users pass between allocState/freeState/distance/interpolate
    nb::class_<ob::State>(m, "State",
        "Opaque OMPL state handle. Use getStateAs() to access typed state.");

    // ========== RealVectorBounds ==========
    nb::class_<ob::RealVectorBounds>(m, "RealVectorBounds",
        "Bounds for real vector state spaces (used for SE2 x,y bounds)")
        .def(nb::init<unsigned int>(), "dim"_a,
            "Create bounds for n dimensions")
        .def("setLow", nb::overload_cast<double>(&ob::RealVectorBounds::setLow), "value"_a,
            "Set lower bound for all dimensions")
        .def("setHigh", nb::overload_cast<double>(&ob::RealVectorBounds::setHigh), "value"_a,
            "Set upper bound for all dimensions")
        .def("setLow", nb::overload_cast<unsigned int, double>(&ob::RealVectorBounds::setLow),
            "index"_a, "value"_a,
            "Set lower bound for specific dimension")
        .def("setHigh", nb::overload_cast<unsigned int, double>(&ob::RealVectorBounds::setHigh),
            "index"_a, "value"_a,
            "Set upper bound for specific dimension")
        .def_rw("low", &ob::RealVectorBounds::low,
            "Lower bounds vector")
        .def_rw("high", &ob::RealVectorBounds::high,
            "Upper bounds vector")
        .def("getVolume", &ob::RealVectorBounds::getVolume,
            "Compute volume of bounded region")
        .def("getDifference", &ob::RealVectorBounds::getDifference,
            "Get high[i] - low[i] for each dimension");

    // ========== SE2StateSpace::StateType ==========
    // SE2State inherits from State so we can pass it to functions expecting State*
    nb::class_<ob::SE2StateSpace::StateType, ob::State>(m, "SE2State",
        "SE(2) state: (x, y, yaw)")
        .def("getX", &ob::SE2StateSpace::StateType::getX, "Get X coordinate")
        .def("getY", &ob::SE2StateSpace::StateType::getY, "Get Y coordinate")
        .def("getYaw", &ob::SE2StateSpace::StateType::getYaw, "Get yaw angle (rotation about Z)")
        .def("setX", &ob::SE2StateSpace::StateType::setX, "x"_a, "Set X coordinate")
        .def("setY", &ob::SE2StateSpace::StateType::setY, "y"_a, "Set Y coordinate")
        .def("setXY", &ob::SE2StateSpace::StateType::setXY, "x"_a, "y"_a, "Set X and Y coordinates")
        .def("setYaw", &ob::SE2StateSpace::StateType::setYaw, "yaw"_a, "Set yaw angle");

    // ========== SE2StateSpace ==========
    nb::class_<ob::SE2StateSpace>(m, "SE2StateSpace",
        "SE(2) state space: position (x,y) and orientation (yaw)")
        .def(nb::init<>())
        .def("setBounds", &ob::SE2StateSpace::setBounds, "bounds"_a,
            "Set bounds for x,y (2D RealVectorBounds)")
        .def("getBounds", &ob::SE2StateSpace::getBounds, nb::rv_policy::reference,
            "Get current x,y bounds")
        .def("allocState", [](ob::SE2StateSpace& ss) -> ob::State* {
            return ss.allocState();
        }, nb::rv_policy::reference,
            "Allocate a new state (must call freeState when done)")
        .def("freeState", [](ob::SE2StateSpace& ss, ob::State* state) {
            ss.freeState(state);
        }, "state"_a,
            "Free a previously allocated state")
        .def("distance", [](ob::SE2StateSpace& ss, ob::State* s1, ob::State* s2) {
            return ss.distance(s1, s2);
        }, "state1"_a, "state2"_a,
            "Compute distance between two states")
        .def("interpolate", [](ob::SE2StateSpace& ss, ob::State* from,
                               ob::State* to, double t, ob::State* state) {
            ss.interpolate(from, to, t, state);
        }, "from"_a, "to"_a, "t"_a, "state"_a,
            "Interpolate: state = from + t*(to-from), t in [0,1]")
        .def("getStateAs", [](ob::SE2StateSpace&, ob::State* s) {
            return s->as<ob::SE2StateSpace::StateType>();
        }, "state"_a, nb::rv_policy::reference,
            "Cast state to SE2State for accessing x,y,yaw");

    // ========== ReedsSheppPathSegmentType ==========
    nb::enum_<ob::ReedsSheppStateSpace::ReedsSheppPathSegmentType>(m, "ReedsSheppPathSegmentType",
        "Segment types in a Reeds-Shepp path")
        .value("RS_NOP", ob::ReedsSheppStateSpace::RS_NOP, "No operation")
        .value("RS_LEFT", ob::ReedsSheppStateSpace::RS_LEFT, "Turn left")
        .value("RS_STRAIGHT", ob::ReedsSheppStateSpace::RS_STRAIGHT, "Go straight")
        .value("RS_RIGHT", ob::ReedsSheppStateSpace::RS_RIGHT, "Turn right");

    // ========== ReedsSheppPath ==========
    nb::class_<ob::ReedsSheppStateSpace::ReedsSheppPath>(m, "ReedsSheppPath",
        "Complete description of a Reeds-Shepp path (up to 5 segments)")
        .def("length", &ob::ReedsSheppStateSpace::ReedsSheppPath::length,
            "Total path length")
        .def_prop_ro("totalLength", [](const ob::ReedsSheppStateSpace::ReedsSheppPath& p) {
            return p.totalLength_;
        }, "Total path length (same as length())")
        .def_prop_ro("lengths", [](const ob::ReedsSheppStateSpace::ReedsSheppPath& p) {
            return std::vector<double>(p.length_, p.length_ + 5);
        }, "Length of each segment (5 values, unused segments are 0)")
        .def_prop_ro("types", [](const ob::ReedsSheppStateSpace::ReedsSheppPath& p) {
            std::vector<int> types(5);
            for (int i = 0; i < 5; i++) types[i] = static_cast<int>(p.type_[i]);
            return types;
        }, "Type of each segment as int (0=NOP, 1=LEFT, 2=STRAIGHT, 3=RIGHT)");

    // ========== ReedsSheppStateSpace ==========
    nb::class_<ob::ReedsSheppStateSpace, ob::SE2StateSpace>(m, "ReedsSheppStateSpace",
        "SE(2) state space with Reeds-Shepp distance metric.\n"
        "For car-like vehicles that can move forward AND backward.")
        .def(nb::init<double>(), "turningRadius"_a = 1.0,
            "Create with specified minimum turning radius")
        .def("distance", [](ob::ReedsSheppStateSpace& ss, ob::State* s1, ob::State* s2) {
            return ss.distance(s1, s2);
        }, "state1"_a, "state2"_a,
            "Reeds-Shepp curve length (not Euclidean distance)")
        .def("interpolate", [](ob::ReedsSheppStateSpace& ss, ob::State* from,
                               ob::State* to, double t, ob::State* state) {
            ss.interpolate(from, to, t, state);
        }, "from"_a, "to"_a, "t"_a, "state"_a,
            "Interpolate along the optimal Reeds-Shepp curve")
        .def("reedsShepp", [](ob::ReedsSheppStateSpace& ss, ob::State* s1, ob::State* s2) {
            return ss.reedsShepp(s1, s2);
        }, "state1"_a, "state2"_a,
            "Compute optimal Reeds-Shepp path between two states");

    // ========== DubinsPathSegmentType ==========
    nb::enum_<ob::DubinsStateSpace::DubinsPathSegmentType>(m, "DubinsPathSegmentType",
        "Segment types in a Dubins path")
        .value("DUBINS_LEFT", ob::DubinsStateSpace::DUBINS_LEFT, "Turn left")
        .value("DUBINS_STRAIGHT", ob::DubinsStateSpace::DUBINS_STRAIGHT, "Go straight")
        .value("DUBINS_RIGHT", ob::DubinsStateSpace::DUBINS_RIGHT, "Turn right");

    // ========== DubinsPath ==========
    nb::class_<ob::DubinsStateSpace::DubinsPath>(m, "DubinsPath",
        "Complete description of a Dubins path (exactly 3 segments)")
        .def("length", &ob::DubinsStateSpace::DubinsPath::length,
            "Total path length")
        .def_prop_ro("lengths", [](const ob::DubinsStateSpace::DubinsPath& p) {
            return std::vector<double>(p.length_, p.length_ + 3);
        }, "Length of each of the 3 segments")
        .def_prop_ro("reverse", [](const ob::DubinsStateSpace::DubinsPath& p) {
            return p.reverse_;
        }, "Whether path should be followed in reverse (for symmetric distance)");

    // ========== DubinsStateSpace ==========
    nb::class_<ob::DubinsStateSpace, ob::SE2StateSpace>(m, "DubinsStateSpace",
        "SE(2) state space with Dubins distance metric.\n"
        "For car-like vehicles that can only move FORWARD (no reverse).\n"
        "Note: Dubins distance is NOT a proper metric (triangle inequality can fail).")
        .def(nb::init<double, bool>(), "turningRadius"_a = 1.0, "isSymmetric"_a = false,
            "Create with turning radius. isSymmetric=true makes d(a,b)=d(b,a) but\n"
            "then triangle inequality may not hold.")
        .def("distance", [](ob::DubinsStateSpace& ss, ob::State* s1, ob::State* s2) {
            return ss.distance(s1, s2);
        }, "state1"_a, "state2"_a,
            "Dubins curve length")
        .def("interpolate", [](ob::DubinsStateSpace& ss, ob::State* from,
                               ob::State* to, double t, ob::State* state) {
            ss.interpolate(from, to, t, state);
        }, "from"_a, "to"_a, "t"_a, "state"_a,
            "Interpolate along the optimal Dubins curve")
        .def("dubins", [](ob::DubinsStateSpace& ss, ob::State* s1, ob::State* s2) {
            return ss.dubins(s1, s2);
        }, "state1"_a, "state2"_a,
            "Compute optimal Dubins path between two states")
        .def("isMetricSpace", &ob::DubinsStateSpace::isMetricSpace,
            "Returns False (Dubins distance is not a proper metric)");
}
