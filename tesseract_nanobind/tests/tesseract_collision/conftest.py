"""Pytest configuration for tesseract_collision tests."""
import pytest


@pytest.fixture(autouse=True)
def cleanup_collision_objects():
    """Cleanup fixture - gc.collect() disabled due to segfault issues.

    Note: gc.collect() in fixture teardown causes segfaults with C++ bindings.
    Tests that need explicit cleanup should handle it in test code with explicit del.
    """
    yield
    # Disabled: gc.collect() causes segfaults with C++ binding cleanup order
    pass
