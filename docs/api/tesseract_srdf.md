# tesseract_robotics.tesseract_srdf

The semantic robot description: groups, group states and TCPs (`KinematicsInformation`), the
allowed collision matrix, collision margins, calibration and the kinematics and contact-manager
plugin configs. `SRDFModel.initFile` / `initString` parse an SRDF against the `SceneGraph` of
its URDF; `saveToFile` writes it back.

```python
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_srdf import SRDFModel
from tesseract_robotics.tesseract_urdf import parseURDFFile

locator = GeneralResourceLocator()
urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf").getFilePath()
srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf").getFilePath()

scene_graph = parseURDFFile(urdf, locator)
model = SRDFModel()
model.initFile(scene_graph, srdf, locator)

model.kinematics_information.group_names       # {'manipulator', 'manipulator_joint_group'}
model.contact_managers_plugin_info              # ContactManagersPluginInfo
model.calibration_info.empty()                  # True: this SRDF has no calibration_config
```

## Named TCPs (`group_tcps`)

A named TCP is a transform with a name, stored per kinematic group. It is not a link: the
physical tip links a group can solve for are `KinematicGroup.getAllPossibleTipLinkNames()`, and
a link is always passed as `ManipulatorInfo.tcp_frame`. A named TCP goes in
`ManipulatorInfo.tcp_offset`, and the TCP pose is

```text
T_world_tool = T_world_{tcp_frame}(q) * findTCPOffset(manip_info)
```

The SRDF records no parent frame for a TCP: its transform is relative to whatever `tcp_frame`
the `ManipulatorInfo` names, so pass the link the offset was measured from (usually the flange,
`tool0`). Nothing checks this.

```xml
<group_tcps group="manipulator">
  <tcp name="laser" xyz="0.01 -0.02 0.25" rpy="0.1 -0.2 0.3"/>
  <tcp name="welder" xyz="0 0.05 0.18" wxyz="0.5 0.5 0.5 0.5"/>
</group_tcps>
```

`xyz`, and one of `rpy` or `wxyz`, are required. `rpy` composes as `Rz(y) Ry(p) Rx(r)`; `wxyz`
is **scalar-first** (upstream's SRDF attribute, unlike the planning API's scalar-last
quaternions) and is normalised. A TCP name is scoped to its group: the same name in two groups
is two independent transforms.

```python
tcps = model.kinematics_information.group_tcps   # dict[str, dict[str, Isometry3d]], a copy
tcps["manipulator"]["laser"].matrix              # 4x4, exactly the parsed transform
model.kinematics_information.hasGroupTCP("manipulator", "laser")
model.kinematics_information.addGroupTCP("manipulator", "probe", offset)
```

`group_tcps` returns a copy: assign the whole mapping back, or use `addGroupTCP` /
`removeGroupTCP`, to change it. `saveToFile` writes each TCP as `xyz` + `wxyz` at 6 significant
digits, so a saved and reloaded TCP differs from the original by up to ~1e-6.

How a planner resolves the name, and the rest of the composition rule, are under
[TCP offsets](tesseract_environment.md#tcp-offsets).

## Plugin and calibration configs

`<kinematics_plugin_config>`, `<contact_managers_plugin_config>` and `<calibration_config>` each
name a YAML file, parsed into `kinematics_information.kinematics_plugin_info`,
`contact_managers_plugin_info` and `calibration_info` (a `tesseract_common.CalibrationInfo`).
`saveToFile(path)` writes each non-empty one as `kinematics_plugin_config.yaml`,
`contact_managers_plugin_config.yaml` and `calibration_config.yaml` beside the SRDF and
references it by that file name, so a saved model reloads equal to itself.

::: tesseract_robotics.tesseract_srdf
    options:
      show_root_heading: true
      show_source: false
      members_order: source
      heading_level: 3
