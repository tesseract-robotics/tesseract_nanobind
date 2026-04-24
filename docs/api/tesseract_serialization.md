# tesseract_robotics.tesseract_serialization

XML and binary serialization for root types. The backend switched from
Boost.Serialization (0.33) to Cereal (0.34); the Python API is unchanged.
See the [Serialization User Guide](../user-guide/serialization.md) for a
walkthrough and the [migration notes](../changes.md) if you have 0.33 archives.

## Free functions

Each supported type exposes paired `to_*` / `from_*` functions for the three
archive formats:

| Type | XML | Binary | File (auto) |
|---|---|---|---|
| `CompositeInstruction` | `composite_instruction_to_xml` / `composite_instruction_from_xml` | `composite_instruction_to_binary` / `composite_instruction_from_binary` | `composite_instruction_to_file` / `composite_instruction_from_file` |
| `Environment` | `environment_to_xml` / `environment_from_xml` | `environment_to_binary` / `environment_from_binary` | `environment_to_file` / `environment_from_file` |
| `SceneState` | `scene_state_to_xml` / `scene_state_from_xml` | `scene_state_to_binary` / `scene_state_from_binary` | `scene_state_to_file` / `scene_state_from_file` |

The `*_to_file` / `*_from_file` helpers auto-detect the format from the path
suffix (`.xml` vs `.bin`).

::: tesseract_robotics.tesseract_serialization
    options:
      show_root_heading: true
      show_source: false
      members_order: source
      heading_level: 3
