"""The failure every planning example raises when its plan is not successful."""

from __future__ import annotations

from typing import Protocol


class PlanResult(Protocol):
    """What `require_success` reads from a `PlanningResult`."""

    successful: bool
    message: str


class PlanningFailedError(RuntimeError):
    """A planning pipeline returned an unsuccessful result."""


def require_success[R: PlanResult](result: R, what: str) -> R:
    """Return `result` if the pipeline succeeded, else raise.

    Args:
        result: Outcome of `TaskComposer.plan`.
        what: Which plan this was, for the message (for example `"PICK"`).

    Returns:
        `result`, unchanged.

    Raises:
        PlanningFailedError: `result.successful` is false; the message carries the
            pipeline's own message.
    """
    if not result.successful:
        raise PlanningFailedError(f"{what} planning failed: {result.message}")
    return result


class TimeParameterizationFailedError(RuntimeError):
    """A time parameterization algorithm rejected the trajectory."""
