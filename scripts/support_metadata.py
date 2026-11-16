"""Build metadata: the runtime dependencies, with the support wheel pinned to this version."""

from collections.abc import Mapping

SUPPORT_DISTRIBUTION = "tesseract-robotics-nanobind-support"


class SupportMetadataError(RuntimeError):
    """The build backend requested an unsupported field or configuration."""


def dynamic_metadata(
    field: str, settings: Mapping[str, object], project: Mapping[str, object]
) -> list[str]:
    """`settings["requires"]` plus the support wheel at the version scikit-build-core resolved.

    Args:
        field: The metadata field requested; only `dependencies` is provided.
        settings: The `[tool.scikit-build.metadata.dependencies]` table minus `provider` and
            `provider-path`; exactly one key, `requires`, a list of requirement strings.
        project: The `[project]` table, with `version` resolvable.

    Returns:
        The `[project] dependencies` list.

    Raises:
        SupportMetadataError: another field, or settings other than a `requires` list.
    """
    requires = settings.get("requires")
    if field != "dependencies" or set(settings) != {"requires"} or not isinstance(requires, list):
        raise SupportMetadataError(f"Unsupported metadata request: {field}, {settings}")
    return [*map(str, requires), f"{SUPPORT_DISTRIBUTION}=={project['version']}"]
