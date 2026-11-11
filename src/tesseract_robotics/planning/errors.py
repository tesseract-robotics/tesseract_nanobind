"""Named exceptions of the planning API, one per failure mode.

Each subclasses the builtin its failure belongs to, so `except ValueError` still catches a bad
argument, while a caller that cares tells the modes apart by type.
"""

from __future__ import annotations


class ResourceNotFoundError(FileNotFoundError):
    """A URDF, SRDF or mesh resource URL or path does not resolve to a file."""


class EnvironmentInitError(RuntimeError):
    """`Environment.init` rejected a URDF / SRDF pair."""


class EnvironmentCommandError(RuntimeError):
    """The environment refused a command (add / remove / move link, collision matrix, margin)."""


class InvalidEnvironmentError(TypeError):
    """`Robot` was handed something that is not an initialised `Environment`."""


class JointNotFoundError(KeyError):
    """A joint name is not among a state's joints."""


class JointNamesRequiredError(ValueError):
    """Joint positions were given without the joint names that order them."""


class JointCountMismatchError(ValueError):
    """Joint positions and joint names differ in length."""


class IKNoSolutionError(RuntimeError):
    """Inverse kinematics found no joint configuration for a pose."""


class EmptyProgramError(ValueError):
    """A motion program has no targets."""


class InvalidTargetError(ValueError):
    """A motion target is built from inconsistent or missing fields."""


class InvalidArrayShapeError(ValueError):
    """A vector or matrix argument does not have the required shape."""


class DegenerateRotationError(ValueError):
    """An axis or quaternion is too short to normalise."""


class MeshLoadError(RuntimeError):
    """A mesh file loaded no geometry."""


class AmbiguousMeshError(ValueError):
    """A mesh file holds several meshes where one geometry is required."""


class ConflictingExecutorError(ValueError):
    """Both `num_threads` and a pre-built executor were given."""


class InvalidThreadCountError(ValueError):
    """A thread count is fewer than 1 thread."""


class UnknownPipelineError(ValueError):
    """A pipeline name is not among the factory's configured task nodes."""


class PipelineLayoutError(RuntimeError):
    """A pipeline lacks the `program` input / output port the composer binds."""


class NoDefaultProfilesError(ValueError):
    """No default profiles are defined for a pipeline; pass `profiles=` explicitly."""


class PipelineWarmupError(RuntimeError):
    """A pipeline named for warmup does not load."""


class ConflictingProfileArgumentsError(ValueError):
    """A full profile override was combined with arguments that configure the same profile."""


class InvalidProfileArgumentError(ValueError):
    """A profile helper argument is out of range (planner count, planning time, ...)."""


class UninitializedEnvironmentError(ValueError):
    """`Robot` was handed an `Environment` that has not been initialised."""
