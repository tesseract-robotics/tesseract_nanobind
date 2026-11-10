"""Build metadata for the support extra, using the binding's resolved SCM version."""

from collections.abc import Mapping


class SupportMetadataError(RuntimeError):
    """The build backend requested an unsupported field or configuration."""


def dynamic_metadata(
    field: str, settings: Mapping[str, object], project: Mapping[str, object]
) -> dict[str, list[str]]:
    """Pin the support wheel to the version resolved by scikit-build-core."""
    if field != "optional-dependencies" or settings:
        raise SupportMetadataError(f"Unsupported metadata request: {field}, {settings}")
    return {"support": [f"tesseract-robotics-nanobind-support=={project['version']}"]}
