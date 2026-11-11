/**
 * @file scene_graph.cpp
 * @brief nanobind bindings for tesseract_scene_graph
 */

#include "tesseract_nb.h"

// tesseract_scene_graph
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>

// tesseract_geometry for Visual/Collision geometry pointer
#include <tesseract/geometry/geometry.h>

// tesseract_common for AllowedCollisionMatrix
#include <tesseract/common/allowed_collision_matrix.h>

#include <sstream>

namespace tsg = tesseract::scene_graph;
namespace tg = tesseract::geometry;

namespace {

// Upstream's operator<< as a string: the `__str__` of every scene graph type that declares one.
template <class T>
std::string stream_str(const T& value) {
    std::ostringstream os;
    os << value;
    return os.str();
}

}  // namespace

NB_MODULE(_scene_graph, m) {
    m.doc() = "tesseract_scene_graph Python bindings";

    // Import common module for the Isometry3d type (else stubs quote the C++ name, which is compiler-specific);
    // LinkId / JointId / LinkIdPair are registered there too
    nb::module_::import_("tesseract_robotics.common._common");

    // ==================== Joint-related classes ====================

    // JointType enum
    nb::enum_<tsg::JointType>(m, "JointType")
        .value("UNKNOWN", tsg::JointType::UNKNOWN)
        .value("REVOLUTE", tsg::JointType::REVOLUTE)
        .value("CONTINUOUS", tsg::JointType::CONTINUOUS)
        .value("PRISMATIC", tsg::JointType::PRISMATIC)
        .value("FLOATING", tsg::JointType::FLOATING)
        .value("PLANAR", tsg::JointType::PLANAR)
        .value("FIXED", tsg::JointType::FIXED)
        // operator<< (joint.h:321): "Revolute", "Fixed", ...; repr() and .name stay Python's
        .def("__str__", &stream_str<tsg::JointType>);

    // JointDynamics
    nb::class_<tsg::JointDynamics>(m, "JointDynamics")
        .def(nb::init<>())
        .def(nb::init<double, double>(), "damping"_a, "friction"_a)
        .def_rw("damping", &tsg::JointDynamics::damping, "Viscous damping [N·s/m or N·m·s/rad].")
        .def_rw("friction", &tsg::JointDynamics::friction, "Static friction [N or N·m].")
        .def("clear", &tsg::JointDynamics::clear, "Reset to zero.")
        .def("__eq__", &tsg::JointDynamics::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::JointDynamics::operator!=, TNB_NE_ATTRS)
        .def("__str__", &stream_str<tsg::JointDynamics>)  // operator<< (joint.h:82)
        .def("__repr__", [](const tsg::JointDynamics& self) {
            return "JointDynamics(damping=" + std::to_string(self.damping) +
                   ", friction=" + std::to_string(self.friction) + ")";
        });

    // JointLimits
    nb::class_<tsg::JointLimits>(m, "JointLimits")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double, double, double>(),
             "lower"_a, "upper"_a, "effort"_a, "velocity"_a, "acceleration"_a, "jerk"_a)
        .def_rw("lower", &tsg::JointLimits::lower, "Lower position limit [m or rad].")
        .def_rw("upper", &tsg::JointLimits::upper, "Upper position limit [m or rad].")
        .def_rw("effort", &tsg::JointLimits::effort, "Effort limit [N or N·m].")
        .def_rw("velocity", &tsg::JointLimits::velocity, "Velocity limit [m/s or rad/s].")
        .def_rw("acceleration", &tsg::JointLimits::acceleration, "Acceleration limit [m/s² or rad/s²].")
        .def_rw("jerk", &tsg::JointLimits::jerk, "Jerk limit [m/s³ or rad/s³].")
        .def("clear", &tsg::JointLimits::clear, "Reset every limit to zero.")
        .def("__eq__", &tsg::JointLimits::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::JointLimits::operator!=, TNB_NE_ATTRS)
        .def("__str__", &stream_str<tsg::JointLimits>)  // operator<< (joint.h:112)
        .def("__repr__", [](const tsg::JointLimits& self) {
            return "JointLimits(lower=" + std::to_string(self.lower) +
                   ", upper=" + std::to_string(self.upper) + ")";
        });

    // JointSafety
    nb::class_<tsg::JointSafety>(m, "JointSafety")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double>(),
             "soft_upper_limit"_a, "soft_lower_limit"_a, "k_position"_a, "k_velocity"_a)
        .def_rw("soft_upper_limit", &tsg::JointSafety::soft_upper_limit, "Upper soft limit [m or rad].")
        .def_rw("soft_lower_limit", &tsg::JointSafety::soft_lower_limit, "Lower soft limit [m or rad].")
        .def_rw("k_position", &tsg::JointSafety::k_position, "Position gain of the safety controller.")
        .def_rw("k_velocity", &tsg::JointSafety::k_velocity, "Velocity gain of the safety controller.")
        .def("clear", &tsg::JointSafety::clear, "Reset to zero.")
        .def("__eq__", &tsg::JointSafety::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::JointSafety::operator!=, TNB_NE_ATTRS)
        .def("__str__", &stream_str<tsg::JointSafety>);  // operator<< (joint.h:173)

    // JointCalibration
    nb::class_<tsg::JointCalibration>(m, "JointCalibration")
        .def(nb::init<>())
        .def(nb::init<double, double, double>(),
             "reference_position"_a, "rising"_a, "falling"_a)
        .def_rw("reference_position", &tsg::JointCalibration::reference_position, "Reference position [m or rad].")
        .def_rw("rising", &tsg::JointCalibration::rising, "Rising-edge position [m or rad].")
        .def_rw("falling", &tsg::JointCalibration::falling, "Falling-edge position [m or rad].")
        .def("clear", &tsg::JointCalibration::clear, "Reset to zero.")
        .def("__eq__", &tsg::JointCalibration::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::JointCalibration::operator!=, TNB_NE_ATTRS)
        .def("__str__", &stream_str<tsg::JointCalibration>);  // operator<< (joint.h:200)

    // JointMimic
    nb::class_<tsg::JointMimic>(m, "JointMimic")
        .def(nb::init<>())
        .def(nb::init<double, double, tesseract::common::JointId>(),
             "offset"_a, "multiplier"_a, "joint_id"_a)
        .def_rw("offset", &tsg::JointMimic::offset, "Offset added after the multiplier [m or rad].")
        .def_rw("multiplier", &tsg::JointMimic::multiplier, "Factor applied to the mimicked joint's value.")
        .def_rw("joint_id", &tsg::JointMimic::joint_id, nb::rv_policy::copy, "The mimicked joint.")
        .def("clear", &tsg::JointMimic::clear, "Reset to offset 0, multiplier 1, no joint.")
        .def("__eq__", &tsg::JointMimic::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::JointMimic::operator!=, TNB_NE_ATTRS)
        .def("__str__", &stream_str<tsg::JointMimic>);  // operator<< (joint.h:227)

    // Joint (non-copyable)
    nb::class_<tsg::Joint>(m, "Joint")
        .def(nb::init<tesseract::common::JointId>(), "id"_a)
        .def(nb::init<>())
        .def("getName", &tsg::Joint::getName, "The joint name.")
        .def("getId", &tsg::Joint::getId, "The joint id.")
        .def_rw("type", &tsg::Joint::type, "The joint type.")
        .def_rw("axis", &tsg::Joint::axis, "The joint axis, in the joint frame (unit vector).")
        .def_rw("child_link_id", &tsg::Joint::child_link_id, nb::rv_policy::copy, "The link this joint moves.")
        .def_rw("parent_link_id", &tsg::Joint::parent_link_id, nb::rv_policy::copy, "The link this joint is attached to.")
        .def_rw("parent_to_joint_origin_transform", &tsg::Joint::parent_to_joint_origin_transform, "The joint frame in the parent link's frame.")
        .def_rw("dynamics", &tsg::Joint::dynamics, "Damping and friction, or None.")
        .def_rw("limits", &tsg::Joint::limits, "Position, velocity, acceleration, jerk and effort limits, or None.")
        .def_rw("safety", &tsg::Joint::safety, "Soft limits, or None.")
        .def_rw("calibration", &tsg::Joint::calibration, "Calibration edges, or None.")
        .def_rw("mimic", &tsg::Joint::mimic, "The joint this one mimics, or None.")
        .def("clear", &tsg::Joint::clear, "Reset every field.")
        .def("clone", nb::overload_cast<>(&tsg::Joint::clone, nb::const_), "A copy, optionally under a new id.")
        .def("clone", nb::overload_cast<tesseract::common::JointId>(&tsg::Joint::clone, nb::const_), "id"_a, "A copy, optionally under a new id.")
        .def("__eq__", &tsg::Joint::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::Joint::operator!=, TNB_NE_ATTRS)
        .def("__repr__", [](const tsg::Joint& self) {
            return "Joint('" + self.getName() + "')";
        });

    // ==================== Link-related classes ====================

    // Material
    nb::class_<tsg::Material>(m, "Material")
        .def(nb::init<>())
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tsg::Material::getName, "The material name.")
        .def_rw("texture_filename", &tsg::Material::texture_filename, "Texture image path, or empty.")
        .def_rw("color", &tsg::Material::color, "RGBA color, each in [0, 1].")
        .def_static("getDefaultMaterial", &tsg::Material::getDefaultMaterial, "The material links without one are drawn with.")
        .def("clear", &tsg::Material::clear, "Reset the color and texture.")
        .def("__eq__", &tsg::Material::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::Material::operator!=, TNB_NE_ATTRS);

    // Inertial
    nb::class_<tsg::Inertial>(m, "Inertial")
        .def(nb::init<>())
        .def_rw("origin", &tsg::Inertial::origin, "The center of mass frame, in the link frame.")
        .def_rw("mass", &tsg::Inertial::mass, "Mass [kg].")
        .def_rw("ixx", &tsg::Inertial::ixx, "Inertia tensor entry [kg·m²], in the origin frame.")
        .def_rw("ixy", &tsg::Inertial::ixy, "Inertia tensor entry [kg·m²], in the origin frame.")
        .def_rw("ixz", &tsg::Inertial::ixz, "Inertia tensor entry [kg·m²], in the origin frame.")
        .def_rw("iyy", &tsg::Inertial::iyy, "Inertia tensor entry [kg·m²], in the origin frame.")
        .def_rw("iyz", &tsg::Inertial::iyz, "Inertia tensor entry [kg·m²], in the origin frame.")
        .def_rw("izz", &tsg::Inertial::izz, "Inertia tensor entry [kg·m²], in the origin frame.")
        .def("clear", &tsg::Inertial::clear, "Reset to zero mass and inertia.")
        .def("__eq__", &tsg::Inertial::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::Inertial::operator!=, TNB_NE_ATTRS);

    // Visual
    nb::class_<tsg::Visual>(m, "Visual")
        .def(nb::init<>())
        .def_rw("origin", &tsg::Visual::origin, "The geometry's frame, in the link frame.")
        .def_prop_rw("geometry",
            [](const tsg::Visual& self) -> std::shared_ptr<tg::Geometry> {
                return std::const_pointer_cast<tg::Geometry>(self.geometry);
            },
            [](tsg::Visual& self, std::shared_ptr<const tg::Geometry> g) { self.geometry = g; }, "The geometry drawn.")
        .def_rw("material", &tsg::Visual::material, "The material, or None for the default.")
        .def_rw("name", &tsg::Visual::name, "The visual's name.")
        .def("clear", &tsg::Visual::clear, "Reset every field.")
        .def("__eq__", &tsg::Visual::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::Visual::operator!=, TNB_NE_ATTRS);

    // Collision
    nb::class_<tsg::Collision>(m, "Collision")
        .def(nb::init<>())
        .def_rw("origin", &tsg::Collision::origin, "The geometry's frame, in the link frame.")
        .def_prop_rw("geometry",
            [](const tsg::Collision& self) -> std::shared_ptr<tg::Geometry> {
                return std::const_pointer_cast<tg::Geometry>(self.geometry);
            },
            [](tsg::Collision& self, std::shared_ptr<const tg::Geometry> g) { self.geometry = g; }, "The collision geometry.")
        .def_rw("name", &tsg::Collision::name, "The collision's name.")
        .def("clear", &tsg::Collision::clear, "Reset every field.")
        .def("__eq__", &tsg::Collision::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::Collision::operator!=, TNB_NE_ATTRS);

    // Link (non-copyable)
    nb::class_<tsg::Link>(m, "Link")
        .def(nb::init<tesseract::common::LinkId>(), "id"_a)
        .def(nb::init<>())
        .def("getName", &tsg::Link::getName, "The link name.")
        .def("getId", &tsg::Link::getId, "The link id.")
        .def_rw("inertial", &tsg::Link::inertial, "Mass properties, or None.")
        .def_rw("visual", &tsg::Link::visual, "The visual elements.")
        .def_rw("collision", &tsg::Link::collision, "The collision elements.")
        .def_rw("visible", &tsg::Link::visible, "Whether the link is visible (link.h:210)")
        .def_rw("collision_enabled", &tsg::Link::collision_enabled,
                "Whether the link takes part in collision checking (link.h:213)")
        // Helper methods to avoid the .append() copy issue with vectors
        // Usage: link.addVisual(visual) instead of link.visual.append(visual)
        .def("addVisual", [](tsg::Link& self, const tsg::Visual& v) {
            self.visual.push_back(std::make_shared<tsg::Visual>(v));
        }, "visual"_a, "Add a Visual to this link")
        .def("addCollision", [](tsg::Link& self, const tsg::Collision& c) {
            self.collision.push_back(std::make_shared<tsg::Collision>(c));
        }, "collision"_a, "Add a Collision to this link")
        .def("clearVisual", [](tsg::Link& self) { self.visual.clear(); },
            "Clear all Visual elements from this link")
        .def("clearCollision", [](tsg::Link& self) { self.collision.clear(); },
            "Clear all Collision elements from this link")
        .def("clear", &tsg::Link::clear, "Reset every field.")
        .def("clone", nb::overload_cast<>(&tsg::Link::clone, nb::const_), "A copy, optionally under a new id.")
        .def("clone", nb::overload_cast<tesseract::common::LinkId>(&tsg::Link::clone, nb::const_), "id"_a, "A copy, optionally under a new id.")
        .def("__eq__", &tsg::Link::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::Link::operator!=, TNB_NE_ATTRS)
        .def("__repr__", [](const tsg::Link& self) {
            return "Link('" + self.getName() + "')";
        });

    // ==================== Graph-related classes ====================

    // ShortestPath
    nb::class_<tsg::ShortestPath>(m, "ShortestPath")
        .def(nb::init<>())
        .def_rw("links", &tsg::ShortestPath::links, nb::rv_policy::copy, "The links on the path, root to tip.")
        .def_rw("joints", &tsg::ShortestPath::joints, nb::rv_policy::copy, "The joints on the path, root to tip.")
        .def_rw("active_joints", &tsg::ShortestPath::active_joints, nb::rv_policy::copy, "The active joints on the path, root to tip.")
        .def("__str__", &stream_str<tsg::ShortestPath>);  // operator<< (graph.h:596)

    // SceneGraph (non-copyable)
    nb::class_<tsg::SceneGraph>(m, "SceneGraph")
        .def(nb::init<const std::string&>(), "name"_a = "")
        .def("clone", &tsg::SceneGraph::clone, "A deep copy.")
        .def("clear", &tsg::SceneGraph::clear, "Remove every link, joint and allowed collision.")
        .def("setName", &tsg::SceneGraph::setName, "name"_a, "Set the graph's name.")
        .def("getName", &tsg::SceneGraph::getName, "The graph's name.")
        .def("setRoot", &tsg::SceneGraph::setRoot, "name"_a, "Make the link `name` the root; False if unknown.")
        .def("getRoot", &tsg::SceneGraph::getRoot, "The root link id.")

        // Link operations
        .def("addLink", nb::overload_cast<const tsg::Link&, bool>(&tsg::SceneGraph::addLink),
             "link"_a, "replace_allowed"_a = false, "Add a link (with the joint attaching it); False if the name exists and replacement is not allowed.")
        .def("addLink", nb::overload_cast<const tsg::Link&, const tsg::Joint&>(&tsg::SceneGraph::addLink),
             "link"_a, "joint"_a, "Add a link (with the joint attaching it); False if the name exists and replacement is not allowed.")
        .def("getLink", &tsg::SceneGraph::getLink, "name"_a, "The link `name` (shared), or None.")
        .def("getLinks", &tsg::SceneGraph::getLinks, "Every link (shared).")
        .def("getLeafLinks", &tsg::SceneGraph::getLeafLinks, "The links with no child joint.")
        .def("removeLink", &tsg::SceneGraph::removeLink, "name"_a, "recursive"_a = false, "Remove the link `name` (and its subtree when `recursive`); False if unknown.")
        .def("moveLink", &tsg::SceneGraph::moveLink, "joint"_a, "Re-attach a link by replacing its parent joint with `joint`; False on failure.")
        .def("setLinkVisibility", &tsg::SceneGraph::setLinkVisibility, "name"_a, "visibility"_a, "Show or hide the link `name`.")
        .def("getLinkVisibility", &tsg::SceneGraph::getLinkVisibility, "name"_a, "Whether the link `name` is visible.")
        .def("setLinkCollisionEnabled", &tsg::SceneGraph::setLinkCollisionEnabled, "name"_a, "enabled"_a, "Enable or disable collision checking of the link `name`.")
        .def("getLinkCollisionEnabled", &tsg::SceneGraph::getLinkCollisionEnabled, "name"_a, "Whether collision checking of the link `name` is enabled.")

        // Joint operations
        .def("addJoint", &tsg::SceneGraph::addJoint, "joint"_a, "Add `joint` between two existing links; False on failure.")
        .def("getJoint", &tsg::SceneGraph::getJoint, "name"_a, "The joint `name` (shared), or None.")
        .def("removeJoint", &tsg::SceneGraph::removeJoint, "name"_a, "recursive"_a = false, "Remove the joint `name` (and its child subtree when `recursive`); False if unknown.")
        .def("moveJoint", &tsg::SceneGraph::moveJoint, "name"_a, "parent_link"_a, "Re-parent the joint `name` onto `parent_link`; False on failure.")
        .def("getJoints", &tsg::SceneGraph::getJoints, "Every joint (shared).")
        .def("getActiveJoints", &tsg::SceneGraph::getActiveJoints, "The active (non-fixed, non-mimic) joints.")
        .def("changeJointOrigin", &tsg::SceneGraph::changeJointOrigin, "name"_a, "new_origin"_a, "Set the origin of the joint `name`; False if unknown.")
        .def("changeJointLimits", &tsg::SceneGraph::changeJointLimits, "name"_a, "limits"_a, "Replace the limits of the joint `name`; False if unknown.")
        .def("changeJointPositionLimits", &tsg::SceneGraph::changeJointPositionLimits,
             "name"_a, "lower"_a, "upper"_a, "Set the position limits of the joint `name`; False if unknown.")
        .def("changeJointVelocityLimits", &tsg::SceneGraph::changeJointVelocityLimits, "name"_a, "limit"_a, "Set the velocity limit of the joint `name`; False if unknown.")
        .def("changeJointAccelerationLimits", &tsg::SceneGraph::changeJointAccelerationLimits, "name"_a, "limit"_a, "Set the acceleration limit of the joint `name`; False if unknown.")
        .def("changeJointJerkLimits", &tsg::SceneGraph::changeJointJerkLimits, "name"_a, "limit"_a, "Set the jerk limit of the joint `name`; False if unknown.")
        .def("getJointLimits", &tsg::SceneGraph::getJointLimits, "name"_a, "The limits of the joint `name` (shared), or None.")

        // Collision matrix
        .def("addAllowedCollision",
             nb::overload_cast<const tesseract::common::LinkId&, const tesseract::common::LinkId&, const std::string&>(
                 &tsg::SceneGraph::addAllowedCollision),
             "link_id1"_a, "link_id2"_a, "reason"_a, "Allow the link pair to collide, with a reason.")
        .def("addAllowedCollision",
             nb::overload_cast<const tesseract::common::LinkIdPair&, const std::string&>(&tsg::SceneGraph::addAllowedCollision),
             "pair"_a, "reason"_a, "Allow the link pair to collide, with a reason.")
        .def("removeAllowedCollision",
             nb::overload_cast<const tesseract::common::LinkId&, const tesseract::common::LinkId&>(
                 &tsg::SceneGraph::removeAllowedCollision),
             "link_id1"_a, "link_id2"_a, "Stop allowing the link pair (or every pair of a link) to collide.")
        .def("removeAllowedCollision",
             nb::overload_cast<const tesseract::common::LinkIdPair&>(&tsg::SceneGraph::removeAllowedCollision),
             "pair"_a, "Stop allowing the link pair (or every pair of a link) to collide.")
        .def("removeAllowedCollision",
             nb::overload_cast<const tesseract::common::LinkId&>(&tsg::SceneGraph::removeAllowedCollision),
             "link_id"_a, "Stop allowing the link pair (or every pair of a link) to collide.")
        .def("clearAllowedCollisions", &tsg::SceneGraph::clearAllowedCollisions, "Remove every allowed collision.")
        .def("isCollisionAllowed", &tsg::SceneGraph::isCollisionAllowed, "pair"_a, "True if the link pair may collide.")
        .def("getAllowedCollisionMatrix",
             nb::overload_cast<>(&tsg::SceneGraph::getAllowedCollisionMatrix),
             "Get the allowed collision matrix")
        // No .none(): upstream stores the pointer unchecked and every later ACM call dereferences
        // it (graph.cpp:746-770 @ 0.35.0), so None is a TypeError here.
        .def("setAllowedCollisionMatrix", &tsg::SceneGraph::setAllowedCollisionMatrix, "acm"_a,
             "Replace the allowed collision matrix. The graph keeps this object: later edits to `acm` "
             "are edits to the graph's matrix, as with getAllowedCollisionMatrix.")

        // Graph queries
        .def("getSourceLink", &tsg::SceneGraph::getSourceLink, "joint_name"_a, "The parent link of the joint `joint_name`.")
        .def("getTargetLink", &tsg::SceneGraph::getTargetLink, "joint_name"_a, "The child link of the joint `joint_name`.")
        .def("getInboundJoints", &tsg::SceneGraph::getInboundJoints, "link_name"_a, "The joints whose child is the link `link_name`.")
        .def("getOutboundJoints", &tsg::SceneGraph::getOutboundJoints, "link_name"_a, "The joints whose parent is the link `link_name`.")
        .def("isAcyclic", &tsg::SceneGraph::isAcyclic, "True if the graph has no cycle.")
        .def("isTree", &tsg::SceneGraph::isTree, "True if the graph is a tree.")
        .def("isEmpty", &tsg::SceneGraph::isEmpty, "True if the graph has no link.")
        .def("getAdjacentLinkIds", &tsg::SceneGraph::getAdjacentLinkIds, "id"_a, "The links one joint below `id`.")
        .def("getInvAdjacentLinkIds", &tsg::SceneGraph::getInvAdjacentLinkIds, "id"_a, "The links one joint above `id`.")
        .def("getLinkChildrenIds", &tsg::SceneGraph::getLinkChildrenIds, "id"_a, "Every link below `id`.")
        .def("getJointChildrenIds",
             nb::overload_cast<const tesseract::common::JointId&>(&tsg::SceneGraph::getJointChildrenIds, nb::const_),
             "id"_a, "Every link below the given joint(s).")
        // after the single-id overload, so a str (itself a sequence of str) never reaches the list form
        .def("getJointChildrenIds",
             nb::overload_cast<const std::vector<tesseract::common::JointId>&>(&tsg::SceneGraph::getJointChildrenIds, nb::const_),
             "ids"_a, "Every link below the given joint(s).")
        .def("getShortestPath", &tsg::SceneGraph::getShortestPath, "root"_a, "tip"_a, "The links and joints from `root` to `tip`.")
        .def("getAdjacencyMap", [](const tsg::SceneGraph& self, const std::vector<tesseract::common::LinkId>& link_ids) {
            // upstream's getVertex throws std::runtime_error for an unknown id (graph.cpp:1083-1090)
            for (const auto& id : link_ids)
                if (!self.getLink(id))
                    throw nb::key_error(("SceneGraph.getAdjacencyMap: no link named '" + id.name() + "'").c_str());
            return self.getAdjacencyMap(link_ids);
        }, "link_ids"_a,
        "Map each link that moves with one of `link_ids` to that link's id (graph.h:499). Each listed "
        "link maps itself and every link below it, stopping at another listed link. An unknown id "
        "raises KeyError.")
        .def("saveDOT", &tsg::SceneGraph::saveDOT, "path"_a, "Write the graph as Graphviz DOT to `path`.")

        // Insert scene graph
        .def("insertSceneGraph",
             nb::overload_cast<const tsg::SceneGraph&, const std::string&>(&tsg::SceneGraph::insertSceneGraph),
             "scene_graph"_a, "prefix"_a = "")
        .def("insertSceneGraph",
             nb::overload_cast<const tsg::SceneGraph&, const tsg::Joint&, const std::string&>(&tsg::SceneGraph::insertSceneGraph),
             "scene_graph"_a, "joint"_a, "prefix"_a = "")

        .def("__eq__", &tsg::SceneGraph::operator==, TNB_EQ_ATTRS)
        .def("__ne__", &tsg::SceneGraph::operator!=, TNB_NE_ATTRS)
        .def("__repr__", [](const tsg::SceneGraph& self) {
            return "SceneGraph('" + self.getName() + "')";
        });

    // Note: SceneState is bound in tesseract_state_solver module with proper
    // AlignedMap to std::map conversion for link_transforms and joint_transforms.
    // Import from tesseract_robotics.state_solver.
}
