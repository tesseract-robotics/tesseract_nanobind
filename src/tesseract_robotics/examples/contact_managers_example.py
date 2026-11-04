"""
Contact Managers Example
========================

Port of upstream `box_box_example.cpp` on every discrete backend, then the
contact-manager plugin factory, collision margins, the allowed collision
matrix, a Python contact validator, `ContactResultMap`, a continuous (cast)
manager and the environment's contact managers.

C++ Reference:
    collision/examples/box_box_example.cpp
    collision/test_suite/include/tesseract/collision/test_suite/collision_box_box_unit.hpp
    collision/test_suite/include/tesseract/collision/test_suite/collision_box_box_cast_unit.hpp
    collision/test/contact_managers_factory_unit.cpp  (PluginFactorAPIUnit)
    common/test/utils_unit.cpp  (TestGetAllowedCollisions)
    support/urdf/contact_manager_plugins.yaml

Overview
--------
1. `ContactManagersPluginFactory(Path, locator)` from `contact_manager_plugins.yaml`;
   list the discrete and continuous plugins.
2. Run `box_box_example` on BulletDiscreteBVH, BulletDiscreteSimple and
   FCLDiscreteBVH: a unit box inside the 2 m convex hull, then 0.1 m outside it
   at margin 0.1 (no contact) and margin 0.25 (distance 0.1).
3. Margin arithmetic: the pair-margin cases of upstream's box-box unit test,
   built with the pair-data constructors, plus `incrementMargins`,
   `scaleMargins` and `apply`; each outcome is checked by a contact test.
4. Add, set-default and remove a plugin (upstream `PluginFactorAPIUnit`),
   `saveConfig` round trip, `clearSearchPaths` / `clearSearchLibraries`.
5. `AllowedCollisionMatrix` from a dict, `getAllowedCollisions`,
   `LinkIdPair`, `removeAllowedCollision(link)`.
6. A `ContactResultValidator` subclass drops one pair; `ContactResultMap`
   iteration, `at`, `filter`, `addContactResult` / `setContactResult`.
7. A continuous manager through the three cast overloads of
   `setCollisionObjectsTransform` (upstream box-box cast unit test).
8. `EnvironmentContactAllowedValidator`, managers by name and the
   `SetActive*ContactManagerCommand` pair on the IIWA environment.

Key Concepts
------------
**PLY loading deviation**:
    Upstream reads `box_2m.ply` with `loadSimplePlyFile`, which is not bound.
    Here it is loaded with `createMeshFromPath` and turned into a hull with
    `makeConvexMesh`, the same call upstream makes on the loaded mesh.

**Margin boundary at x = 1.6**:
    Both backends keep a contact whose distance is at most the margin
    (`bullet_utils.cpp:1072`, `fcl_utils.cpp:307`). The gap is
    `(1.6 - 0.5) - 1.0 = 0.10000000000000009` in doubles, one ulp above 0.1, so
    margin 0.1 reports nothing, as upstream's unit test expects
    (`collision_box_box_unit.hpp:201`).

**What the backends agree on**:
    They agree on the colliding pair set. Penetration depth is backend-defined:
    FCL reports the analytic 1.3 m, Bullet 1.266 m on this hull (upstream's own
    unit test avoids `box_2m.ply` for Bullet, `collision_box_box_unit.hpp:58`).
    Bullet's simple manager also reports `link_ids` in the other order, so pairs
    are compared as `ContactResultMap` keys: `LinkIdPair` is unordered, and
    `orderedNameView()` gives its names sorted.
"""

import math
import tempfile
from pathlib import Path
from typing import Any, override

import numpy as np

from tesseract_robotics.tesseract_collision import (
    ContactManagersPluginFactory,
    ContactRequest,
    ContactResult,
    ContactResultMap,
    ContactResultValidator,
    ContactResultVector,
    ContactTestType,
    makeConvexMesh,
)
from tesseract_robotics.tesseract_common import (
    AllowedCollisionMatrix,
    CollisionMarginData,
    CollisionMarginPairData,
    CollisionMarginPairOverrideType,
    GeneralResourceLocator,
    Isometry3d,
    LinkIdPair,
    PluginInfo,
    getAllowedCollisions,
)
from tesseract_robotics.tesseract_environment import (
    Environment,
    EnvironmentContactAllowedValidator,
    SetActiveContinuousContactManagerCommand,
    SetActiveDiscreteContactManagerCommand,
)
from tesseract_robotics.tesseract_geometry import Box, createMeshFromPath

PLUGINS_URL = "package://tesseract/support/urdf/contact_manager_plugins.yaml"
BOX_2M_URL = "package://tesseract/support/meshes/box_2m.ply"
IIWA_URDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
IIWA_SRDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf"

BOX_PAIR = ("box_link", "second_box_link")
# box_box_example.cpp: margins (m) and box_link poses
MARGIN = 0.1
LARGER_MARGIN = 0.25
PENETRATING_XY = (0.2, 0.1)
GAP_X = 1.6
# Gap between box_link (half width 0.5 at x = 1.6) and the 2 m hull (half width 1), m
BOX_GAP = GAP_X - 0.5 - 1.0
# Distance / point tolerance of upstream's box-box unit tests, m
# (collision_box_box_unit.hpp:162, 243, 280; collision_box_box_cast_unit.hpp:166-188)
BOX_BOX_TOL = 0.001
# Margins built by increment / scale round twice (forming 0.25 - 0.1 or 0.25 / 0.1,
# then applying it), each at most one ulp of 0.25 m: m
MARGIN_ARITHMETIC_TOL = 2 * math.ulp(LARGER_MARGIN)

# collision_box_box_unit.hpp:208: a margin on a pair that is not in the scene
UNRELATED_PAIR = LinkIdPair("not_box_link", "also_not_box_link")
UNRELATED_MARGIN = 1.7

# collision_box_box_cast_unit.hpp: static box, moving box (shape offset), sweep
CAST_MOVING_SIZE = 0.25
CAST_MOVING_OFFSET = (0.5, -0.5, 0.0)
CAST_START = (-1.9, 0.0, 0.0)
CAST_END = (1.9, 3.8, 0.0)

# utils_unit.cpp TestGetAllowedCollisions entries and query
ACM_ENTRIES = [
    ("link1", "link2"),
    ("link4", "link3"),
    ("link1", "link3"),
    ("link1", "cause_duplicate"),
    ("link10", "link11"),
    ("dummy", "dummy"),
]
ACM_REASONS = ["test", "test", "test", "test", "test", "link1"]
ACM_QUERY = ["link1", "link3", "cause_duplicate"]


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return Path(resource.getFilePath())


def translation(x: float, y: float = 0.0, z: float = 0.0) -> Isometry3d:
    """A pure translation."""
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return Isometry3d(matrix)


def contact_pairs(results: ContactResultMap) -> list[tuple[str, str]]:
    """Sorted link-name pairs that hold at least one contact."""
    return sorted(key.orderedNameView() for key, vector in results if len(vector))


def contact_test(manager, test_type: ContactTestType = ContactTestType.CLOSEST, request=None):
    """Run a contact test and return the result map."""
    results = ContactResultMap()
    manager.contactTest(results, request if request is not None else ContactRequest(test_type))
    return results


def add_box_box_objects(manager, hull) -> None:
    """box_box_example.cpp: the unit box, the disabled thin box and the 2 m hull."""
    manager.addCollisionObject("box_link", 0, [Box(1, 1, 1)], [translation(0)])
    manager.addCollisionObject("thin_box_link", 0, [Box(0.1, 1, 1)], [translation(0)], False)
    manager.addCollisionObject("second_box_link", 0, [hull], [translation(0)])
    manager.setActiveCollisionObjects(list(BOX_PAIR))


def box_box(manager, hull) -> dict[str, Any]:
    """box_box_example.cpp on one discrete manager."""
    # --8<-- [start:box_box]
    add_box_box_objects(manager, hull)
    manager.setCollisionMarginData(CollisionMarginData(MARGIN))

    manager.setCollisionObjectsTransform(
        {"box_link": translation(*PENETRATING_XY), "second_box_link": translation(0)}
    )
    penetrating = contact_test(manager)

    manager.setCollisionObjectsTransform({"box_link": translation(GAP_X)})
    at_margin = contact_test(manager)

    manager.setDefaultCollisionMargin(LARGER_MARGIN)
    at_larger_margin = contact_test(manager)
    # --8<-- [end:box_box]
    return {
        "manager_name": manager.getName(),
        "penetrating_pairs": contact_pairs(penetrating),
        "penetration_distance": penetrating.at(BOX_PAIR)[0].distance,
        "gap_pairs_at_margin_0_1": contact_pairs(at_margin),
        "gap_distance_at_margin_0_25": at_larger_margin.at(BOX_PAIR)[0].distance,
    }


def margin_outcome(manager, data: CollisionMarginData) -> dict[str, Any]:
    """Apply `data` at the gap pose and report its maxima and the contact outcome."""
    manager.setCollisionMarginData(data)
    return {
        "max": data.getMaxCollisionMargin(),
        "max_box_link": data.getMaxCollisionMargin("box_link"),
        "contact": contact_pairs(contact_test(manager)) == [BOX_PAIR],
    }


def margins(manager) -> dict[str, Any]:
    """Margin arithmetic on the gap pose (x = 1.6, gap 0.1 m)."""
    # --8<-- [start:margins]
    unrelated = CollisionMarginPairData({UNRELATED_PAIR: UNRELATED_MARGIN})
    box_pair = CollisionMarginPairData({BOX_PAIR: LARGER_MARGIN})

    cases = {
        "unrelated_pair_1_7": CollisionMarginData(MARGIN, unrelated),
        "box_pair_0_25": CollisionMarginData(MARGIN, box_pair),
        "pair_only": CollisionMarginData(box_pair),
    }
    incremented = CollisionMarginData(MARGIN)
    incremented.incrementMargins(LARGER_MARGIN - MARGIN)
    scaled = CollisionMarginData(MARGIN)
    scaled.scaleMargins(LARGER_MARGIN / MARGIN)
    modify_applied = CollisionMarginData(MARGIN, unrelated)
    modify_applied.apply(box_pair, CollisionMarginPairOverrideType.MODIFY)
    replace_applied = CollisionMarginData(MARGIN, unrelated)
    replace_applied.apply(box_pair, CollisionMarginPairOverrideType.REPLACE)
    # --8<-- [end:margins]

    result: dict[str, Any] = {name: margin_outcome(manager, data) for name, data in cases.items()}
    for name, data in [
        ("incremented", incremented),
        ("scaled", scaled),
        ("modify_applied", modify_applied),
    ]:
        result[name] = {
            **margin_outcome(manager, data),
            "box_pair_margin": data.getCollisionMargin(*BOX_PAIR),
        }
    result["replace_applied"] = margin_outcome(manager, replace_applied)

    # --8<-- [start:pair_data]
    pair_incremented = CollisionMarginPairData({BOX_PAIR: MARGIN})
    pair_incremented.incrementMargins(LARGER_MARGIN - MARGIN)
    pair_scaled = CollisionMarginPairData({BOX_PAIR: MARGIN})
    pair_scaled.scaleMargins(LARGER_MARGIN / MARGIN)
    pair_modified = CollisionMarginPairData({BOX_PAIR: MARGIN})
    pair_modified.apply(unrelated, CollisionMarginPairOverrideType.MODIFY)
    pair_replaced = CollisionMarginPairData({BOX_PAIR: MARGIN})
    pair_replaced.apply(unrelated, CollisionMarginPairOverrideType.REPLACE)
    # --8<-- [end:pair_data]
    result["pair_data"] = {
        "incremented": pair_incremented.getMaxCollisionMargin(),
        "scaled": pair_scaled.getMaxCollisionMargin(),
        "max_box_link": pair_modified.getMaxCollisionMargin("box_link"),
        "max_unrelated_link": pair_incremented.getMaxCollisionMargin("not_box_link"),
        "modify_applied_pairs": sorted(
            p.orderedNameView() for p in pair_modified.getCollisionMargins()
        ),
        "replace_applied_pairs": sorted(
            p.orderedNameView() for p in pair_replaced.getCollisionMargins()
        ),
    }
    return result


def copy_plugin_info(info: PluginInfo) -> PluginInfo:
    """A new `PluginInfo` with the class and config of `info`."""
    copy = PluginInfo()
    copy.class_name = info.class_name
    copy.config = info.config
    return copy


def plugin_edits(factory: ContactManagersPluginFactory) -> dict[str, dict[str, Any]]:
    """PluginFactorAPIUnit: add a copy of a plugin, make it the default, create it, remove it."""
    # --8<-- [start:edit_plugins]
    simple = factory.getDiscreteContactManagerPlugins()["BulletDiscreteSimpleManager"]
    factory.addDiscreteContactManagerPlugin("TestDiscreteManager", copy_plugin_info(simple))
    factory.setDefaultDiscreteContactManagerPlugin("TestDiscreteManager")
    discrete_default = factory.getDefaultDiscreteContactManagerPlugin()
    discrete_created = factory.createDiscreteContactManager(discrete_default).getName()
    factory.removeDiscreteContactManagerPlugin("TestDiscreteManager")

    cast_simple = factory.getContinuousContactManagerPlugins()["BulletCastSimpleManager"]
    factory.addContinuousContactManagerPlugin(
        "TestContinuousManager", copy_plugin_info(cast_simple)
    )
    factory.setDefaultContinuousContactManagerPlugin("TestContinuousManager")
    continuous_default = factory.getDefaultContinuousContactManagerPlugin()
    continuous_created = factory.createContinuousContactManager(continuous_default).getName()
    factory.removeContinuousContactManagerPlugin("TestContinuousManager")
    # --8<-- [end:edit_plugins]
    return {
        "discrete": {
            "default_after_set": discrete_default,
            "created_name": discrete_created,
            "after_remove": list(factory.getDiscreteContactManagerPlugins()),
            "default_after_remove": factory.getDefaultDiscreteContactManagerPlugin(),
        },
        "continuous": {
            "default_after_set": continuous_default,
            "created_name": continuous_created,
            "after_remove": list(factory.getContinuousContactManagerPlugins()),
            "default_after_remove": factory.getDefaultContinuousContactManagerPlugin(),
        },
    }


def allowed_collisions() -> dict[str, Any]:
    """TestGetAllowedCollisions, with the ACM built from a dict."""
    # --8<-- [start:acm]
    entries = {
        LinkIdPair(*link_pair): reason for link_pair, reason in zip(ACM_ENTRIES, ACM_REASONS)
    }
    acm = AllowedCollisionMatrix(entries)

    added = AllowedCollisionMatrix()
    for (link_1, link_2), reason in zip(ACM_ENTRIES, ACM_REASONS):
        added.addAllowedCollision(link_1, link_2, reason)

    from_dict_equal = acm == added

    allowed_deduplicated = getAllowedCollisions(ACM_QUERY, acm.getAllAllowedCollisions(), True)
    allowed_with_duplicates = getAllowedCollisions(ACM_QUERY, acm.getAllAllowedCollisions(), False)

    acm.removeAllowedCollision("link1")
    # --8<-- [end:acm]
    return {
        "from_dict_equal": from_dict_equal,
        "allowed_deduplicated": [str(link_id) for link_id in allowed_deduplicated],
        "allowed_with_duplicates": allowed_with_duplicates,
        "ordered_pair": LinkIdPair("link4", "link3").orderedNameView(),
        "after_remove_link1": sorted(p.orderedNameView() for p in acm.getAllAllowedCollisions()),
        "str_after_remove": str(acm),
    }


# --8<-- [start:validator_class]
class DropPairValidator(ContactResultValidator):
    """Rejects every contact between two given links; records what it was asked."""

    def __init__(self, link_pair: LinkIdPair):
        super().__init__()
        self.link_pair = link_pair
        self.seen: list[tuple[str, str]] = []

    @override
    def __call__(self, result: ContactResult) -> bool:
        link_pair = LinkIdPair(*result.link_ids)
        self.seen.append(link_pair.orderedNameView())
        return link_pair != self.link_pair


# --8<-- [end:validator_class]


def validator_and_result_map(manager, hull) -> dict[str, Any]:
    """Three overlapping objects: drop one pair with a validator, then edit the result map."""
    add_box_box_objects(manager, hull)
    manager.setCollisionMarginData(CollisionMarginData(MARGIN))
    manager.setCollisionObjectsTransform(
        {"box_link": translation(*PENETRATING_XY), "second_box_link": translation(0)}
    )
    # --8<-- [start:validator]
    manager.enableCollisionObject("thin_box_link")
    manager.setActiveCollisionObjects(["box_link", "thin_box_link", "second_box_link"])

    dropped = LinkIdPair("thin_box_link", "box_link")
    validator = DropPairValidator(dropped)
    probe = ContactResult()
    probe.link_ids = list(dropped.orderedNameView())
    dropped_pair = validator(probe)
    probe.link_ids = list(BOX_PAIR)
    kept_pair = validator(probe)
    validator.seen.clear()

    without = contact_test(manager, ContactTestType.ALL)
    request = ContactRequest(ContactTestType.ALL)
    request.is_valid = validator
    with_validator = contact_test(manager, request=request)
    # --8<-- [end:validator]

    # --8<-- [start:result_map]
    iterated_keys = [key.orderedNameView() for key, _ in without]
    container = without.getContainer()
    at_matches_container = all(
        len(without.at(key)) == len(vector) for key, vector in container.items()
    )

    def keep_thin_box_with_box(key, vector):
        if key != dropped:
            vector.clear()

    filtered = contact_test(manager, ContactTestType.ALL)
    filtered.filter(keep_thin_box_with_box)

    edited = contact_test(manager, ContactTestType.ALL)
    counts = {"start": edited.count()}
    edited.addContactResult(BOX_PAIR, edited.at(BOX_PAIR)[0])
    counts["add_result"] = edited.count()
    two = ContactResultVector()
    two.append(edited.at(BOX_PAIR)[0])
    two.append(edited.at(BOX_PAIR)[0])
    edited.addContactResult(BOX_PAIR, two)
    counts["add_vector"] = edited.count()
    edited.clear()
    edited.setContactResult(BOX_PAIR, two[0])
    counts["set_result"] = edited.count()
    edited.setContactResult(BOX_PAIR, two)
    counts["set_vector"] = edited.count()
    # --8<-- [end:result_map]

    return {
        "validator": {
            "direct_call": {"dropped_pair": dropped_pair, "kept_pair": kept_pair},
            "pairs_without": contact_pairs(without),
            "pairs_with": contact_pairs(with_validator),
            "seen": sorted(validator.seen),
        },
        "result_map": {
            "iterated_keys": sorted(iterated_keys),
            "at_matches_container": at_matches_container,
            "after_filter": contact_pairs(filtered),
            "counts": counts,
        },
    }


def add_cast_objects(manager) -> None:
    """collision_box_box_cast_unit.hpp: static unit box, disabled thin box, moving box."""
    manager.addCollisionObject("static_box_link", 0, [Box(1, 1, 1)], [translation(0)])
    manager.addCollisionObject("thin_box_link", 0, [Box(0.1, 1, 1)], [translation(0)], False)
    moving = Box(CAST_MOVING_SIZE, CAST_MOVING_SIZE, CAST_MOVING_SIZE)
    manager.addCollisionObject("moving_box_link", 0, [moving], [translation(*CAST_MOVING_OFFSET)])
    manager.setActiveCollisionObjects(["moving_box_link"])
    manager.setDefaultCollisionMargin(MARGIN)
    manager.setCollisionObjectsTransform(["static_box_link"], [translation(0)])


def cast_result(manager) -> dict[str, Any]:
    """The upstream cast-unit quantities of the closest swept contact."""
    contact = contact_test(manager).at(("moving_box_link", "static_box_link"))[0]
    # transform / cc_transform are the moving box's start and end poses;
    # nearest_points_local is in its link frame
    local = contact.nearest_points_local[1]
    return {
        "distance": contact.distance,
        "cc_time": list(contact.cc_time),
        "cc_type": [cc_type.name for cc_type in contact.cc_type],
        "swept_start_point": contact.transform[1] * local,
        "swept_end_point": contact.cc_transform[1] * local,
        "subshape_id": list(contact.subshape_id),
    }


def continuous(manager) -> dict[str, dict[str, Any]]:
    """Each cast overload, starting from the box parked at the start pose.

    Parking is a zero-length sweep: a single-pose `setCollisionObjectsTransform`
    on a cast manager moves the object but keeps its previous sweep
    (`bullet_cast_bvh_manager.cpp:240-256`).
    """
    add_cast_objects(manager)
    start, end = translation(*CAST_START), translation(*CAST_END)

    def park() -> int:
        manager.setCollisionObjectsTransform("moving_box_link", start, start)
        return contact_test(manager).count()

    result = {}
    stationary = park()
    # --8<-- [start:cast]
    manager.setCollisionObjectsTransform("moving_box_link", start, end)
    # --8<-- [end:cast]
    result["name"] = {"stationary_contacts": stationary, **cast_result(manager)}

    stationary = park()
    # --8<-- [start:cast_names]
    manager.setCollisionObjectsTransform(["moving_box_link"], [start], [end])
    # --8<-- [end:cast_names]
    result["names"] = {"stationary_contacts": stationary, **cast_result(manager)}

    stationary = park()
    # --8<-- [start:cast_maps]
    manager.setCollisionObjectsTransform({"moving_box_link": start}, {"moving_box_link": end})
    # --8<-- [end:cast_maps]
    result["maps"] = {"stationary_contacts": stationary, **cast_result(manager)}
    return result


def environment(locator: GeneralResourceLocator) -> dict[str, Any]:
    """The IIWA environment's contact managers and its contact-allowed validator."""
    env = Environment()
    if not env.init(locate(locator, IIWA_URDF_URL), locate(locator, IIWA_SRDF_URL), locator):
        raise RuntimeError("IIWA environment failed to initialise")
    scene_graph = env.getSceneGraph()

    # --8<-- [start:environment]
    validator = EnvironmentContactAllowedValidator(scene_graph)
    links = [link.getId() for link in scene_graph.getLinks()]
    validator_matches_scene_graph = all(
        validator(LinkIdPair(a, b)) == scene_graph.isCollisionAllowed(LinkIdPair(a, b))
        for a in links
        for b in links
    )

    by_name = (
        env.getDiscreteContactManager("FCLDiscreteBVHManager").getName(),
        env.getContinuousContactManager("BulletCastSimpleManager").getName(),
    )

    discrete_command = SetActiveDiscreteContactManagerCommand("BulletDiscreteSimpleManager")
    continuous_command = SetActiveContinuousContactManagerCommand("BulletCastSimpleManager")
    applied = (env.applyCommand(discrete_command), env.applyCommand(continuous_command))
    active_after_commands = (
        env.getDiscreteContactManager().getName(),
        env.getContinuousContactManager().getName(),
    )
    # --8<-- [end:environment]
    return {
        "validator_matches_scene_graph": validator_matches_scene_graph,
        # lbr_iiwa_14_r820.srdf:53 disables base_link / link_1 ("Adjacent");
        # base_link / link_5 has no disable_collisions entry
        "adjacent_allowed": validator(LinkIdPair("base_link", "link_1")),
        "distant_allowed": validator(LinkIdPair("base_link", "link_5")),
        "by_name": by_name,
        "commands": (discrete_command.getName(), continuous_command.getName()),
        "applied": applied,
        "active_after_commands": active_after_commands,
    }


def run() -> dict[str, Any]:
    """Run the example; return every result the test asserts on."""
    # --8<-- [start:factory]
    locator = GeneralResourceLocator()
    factory = ContactManagersPluginFactory(locate(locator, PLUGINS_URL), locator)

    discrete_plugins = list(factory.getDiscreteContactManagerPlugins())
    continuous_plugins = list(factory.getContinuousContactManagerPlugins())
    defaults = (
        factory.getDefaultDiscreteContactManagerPlugin(),
        factory.getDefaultContinuousContactManagerPlugin(),
    )
    # --8<-- [end:factory]

    # --8<-- [start:hull]
    mesh = createMeshFromPath(str(locate(locator, BOX_2M_URL)))[0]
    hull = makeConvexMesh(mesh)
    # --8<-- [end:hull]

    box_box_results = {
        name: box_box(factory.createDiscreteContactManager(name), hull) for name in discrete_plugins
    }

    # --8<-- [start:plugin_info]
    discrete_info = factory.getDiscreteContactManagerPlugins()["BulletDiscreteSimpleManager"]
    continuous_info = factory.getContinuousContactManagerPlugins()["BulletCastSimpleManager"]
    created_by_plugin_info = (
        factory.createDiscreteContactManager(
            "BulletDiscreteSimpleManager", discrete_info
        ).getName(),
        factory.createContinuousContactManager(
            "BulletCastSimpleManager", continuous_info
        ).getName(),
    )
    # --8<-- [end:plugin_info]

    margin_manager = factory.createDiscreteContactManager(defaults[0])
    add_box_box_objects(margin_manager, hull)
    margin_manager.setCollisionObjectsTransform(
        {"box_link": translation(GAP_X), "second_box_link": translation(0)}
    )
    margin_results = margins(margin_manager)

    edits = plugin_edits(factory)

    # --8<-- [start:save_config]
    with tempfile.TemporaryDirectory() as tmp:
        saved = Path(tmp) / "contact_manager_plugins.yaml"
        factory.saveConfig(saved)
        config_round_trip_equal = (
            ContactManagersPluginFactory(saved, locator).getConfig() == factory.getConfig()
        )
    # --8<-- [end:save_config]

    validator_results = validator_and_result_map(
        factory.createDiscreteContactManager(defaults[0]), hull
    )
    continuous_results = continuous(factory.createContinuousContactManager(defaults[1]))

    # --8<-- [start:clear_search]
    factory.clearSearchPaths()
    factory.clearSearchLibraries()
    cleared_search = {"paths": factory.getSearchPaths(), "libraries": factory.getSearchLibraries()}
    # --8<-- [end:clear_search]

    return {
        "discrete_plugins": discrete_plugins,
        "continuous_plugins": continuous_plugins,
        "defaults": defaults,
        "created_by_plugin_info": created_by_plugin_info,
        "hull_vertex_count": hull.getVertexCount(),
        "box_box": box_box_results,
        "margins": margin_results,
        "plugin_edits": edits,
        "config_round_trip_equal": config_round_trip_equal,
        "cleared_search": cleared_search,
        "acm": allowed_collisions(),
        **validator_results,
        "continuous": continuous_results,
        "environment": environment(locator),
    }


def main() -> None:
    """Console entry point (`tesseract_contact_managers_example`): run() and print the results."""
    result = run()
    print(f"Discrete managers: {result['discrete_plugins']}")
    print(f"Continuous managers: {result['continuous_plugins']} (defaults {result['defaults']})")
    print(f"Created from PluginInfo: {result['created_by_plugin_info']}")
    print(f"box_2m.ply hull: {result['hull_vertex_count']} vertices")
    for name, box_box_result in result["box_box"].items():
        print(
            f"{name}: inside -> {box_box_result['penetrating_pairs']} "
            f"(distance {box_box_result['penetration_distance']:.4f}); "
            f"x = {GAP_X}, margin {MARGIN} -> {box_box_result['gap_pairs_at_margin_0_1']}; "
            f"margin {LARGER_MARGIN} -> distance {box_box_result['gap_distance_at_margin_0_25']:.4f}"
        )
    for name, outcome in result["margins"].items():
        print(f"margins {name}: {outcome}")
    for kind, edits in result["plugin_edits"].items():
        print(
            f"{kind}: default {edits['default_after_set']} creates {edits['created_name']}; "
            f"after remove {edits['after_remove']}, default {edits['default_after_remove']}"
        )
    print(f"saveConfig round trip equal: {result['config_round_trip_equal']}")
    print(f"After clearSearchPaths / clearSearchLibraries: {result['cleared_search']}")
    acm = result["acm"]
    print(f"Allowed with {ACM_QUERY}: {sorted(acm['allowed_deduplicated'])}")
    print(f"ACM after removeAllowedCollision('link1'): {acm['after_remove_link1']}")
    validator = result["validator"]
    print(f"Contacts without validator: {validator['pairs_without']}")
    print(f"Contacts with validator:    {validator['pairs_with']}")
    print(f"ContactResultMap counts: {result['result_map']['counts']}")
    for overload, cast in result["continuous"].items():
        print(
            f"cast ({overload}): distance {cast['distance']:.4f}, cc_time {cast['cc_time']}, "
            f"start point {np.round(cast['swept_start_point'], 4)}, "
            f"end point {np.round(cast['swept_end_point'], 4)}"
        )
    env = result["environment"]
    print(f"Environment managers by name: {env['by_name']}")
    print(f"Active after SetActive*ContactManagerCommand: {env['active_after_commands']}")


if __name__ == "__main__":
    main()
