# tesseract_robotics.tesseract_serialization

XML and binary serialization for root types. The backend switched from
Boost.Serialization (0.33) to Cereal (0.34); the Python API is unchanged.
See the [Serialization User Guide](../user-guide/serialization.md) for a
walkthrough and the [migration notes](../changes.md) if you have 0.33 archives.

## Free functions

Each supported type exposes paired `to_*` / `from_*` functions for the three
archive formats:

| Type | XML | Binary | File (XML) |
|---|---|---|---|
| `CompositeInstruction` | `composite_instruction_to_xml` / `composite_instruction_from_xml` | `composite_instruction_to_binary` / `composite_instruction_from_binary` | `composite_instruction_to_file` / `composite_instruction_from_file` |
| `Environment` | `environment_to_xml` / `environment_from_xml` | `environment_to_binary` / `environment_from_binary` | `environment_to_file` / `environment_from_file` |
| `SceneState` | `scene_state_to_xml` / `scene_state_from_xml` | `scene_state_to_binary` / `scene_state_from_binary` | `scene_state_to_file` / `scene_state_from_file` |
| `TaskComposerLog` | `task_composer_log_to_xml` / `task_composer_log_from_xml` | `task_composer_log_to_binary` / `task_composer_log_from_binary` | `task_composer_log_to_file` / `task_composer_log_from_file` |

The `*_to_file` / `*_from_file` helpers always write and read XML, whatever the
path suffix: there is no format detection. For a binary file, write the bytes of
`*_to_binary` yourself. `task_composer_log_to_file` / `task_composer_log_from_file`
take a `str` or an `os.PathLike` (`pathlib.Path`); the other `*_file` helpers take
a `str`.

!!! warning "A `TaskComposerLog` holding planner profiles cannot be serialized"
    The planner profile types (TrajOpt, OMPL, Descartes, ...) are not registered
    with this module's cereal registry in the 0.35 bindings. Serializing a log
    whose `initial_data` or `context` holds a `ProfileDictionary` with planner
    profiles raises `RuntimeError` ("Trying to save an unregistered polymorphic
    type"). Store an empty `ProfileDictionary` in the log and rebuild the
    profiles when you replay it.

::: tesseract_robotics.tesseract_serialization
    options:
      show_root_heading: true
      show_source: false
      members_order: source
      heading_level: 3
