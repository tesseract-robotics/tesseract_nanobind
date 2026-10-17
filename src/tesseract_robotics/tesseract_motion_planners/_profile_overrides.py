"""Refuse hook overrides the planner would never call.

Only the abstract profile bases (`SimplePlannerMoveProfile`, `TrajOptIfoptMoveProfile`,
`TrajOptIfoptCompositeProfile`, `DescartesMoveProfileD`) are bound with a trampoline, the C++
shim that forwards a virtual call to a Python override. The concrete profiles are not: a Python
subclass of `SimplePlannerLVSMoveProfile` that defines `generate` is constructed as the plain C++
profile, and the planner calls the C++ `generate`. The override is silently ignored, so the
subclass is refused when the class statement runs.

The guard sees only hooks in a class body. It misses a hook assigned to the class after its
definition or set on an instance, and a subclass of an intermediate class whose own
`__init_subclass__` does not call `super().__init_subclass__()`.
"""

from __future__ import annotations


class ProfileOverrideNotDispatchedError(TypeError):
    """A Python subclass of a concrete profile defines a hook the planner never calls on it."""


# Public home: tesseract_motion_planners re-exports it, as do the three planner packages.
ProfileOverrideNotDispatchedError.__module__ = "tesseract_robotics.tesseract_motion_planners"


def refuse_hook_overrides(concrete: type, hooks: tuple[str, ...], instead: str) -> None:
    """Make defining any of `hooks` in a Python subclass of `concrete` raise.

    Args:
        concrete: A bound concrete profile class, which has no trampoline.
        hooks: The virtual hooks the planner calls on it.
        instead: What to do instead, for the error message.

    Raises:
        ProfileOverrideNotDispatchedError: at class definition, from the installed hook.
    """

    def __init_subclass__(cls: type, **kwargs: object) -> None:
        overridden = sorted(hook for hook in hooks if hook in cls.__dict__)
        if overridden:
            raise ProfileOverrideNotDispatchedError(
                f"{cls.__qualname__} overrides {', '.join(overridden)} on {concrete.__name__}, "
                f"which the planner would never call: {concrete.__name__} has no Python dispatch. "
                f"{instead}"
            )
        super(concrete, cls).__init_subclass__(**kwargs)  # type: ignore[misc]

    # Named as the class's own method, so the stub generator renders it in the class instead of
    # importing a private helper.
    __init_subclass__.__module__ = concrete.__module__
    __init_subclass__.__qualname__ = f"{concrete.__qualname__}.__init_subclass__"
    concrete.__init_subclass__ = classmethod(__init_subclass__)  # type: ignore[method-assign,assignment]
