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
