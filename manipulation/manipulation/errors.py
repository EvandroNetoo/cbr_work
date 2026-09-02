"""Stable domain errors mapped to manipulation action result codes."""


class ManipulationError(RuntimeError):
    """Base class for expected manipulation failures."""


class OperationCanceled(ManipulationError):
    pass


class ConfigurationError(ManipulationError):
    pass


class ServerUnavailable(ManipulationError):
    pass


class PerceptionUnavailable(ManipulationError):
    pass


class NoFreeSpace(ManipulationError):
    pass


class FeatureUnavailable(ManipulationError):
    pass


class ObjectNotFound(ManipulationError):
    pass


class MotionFailed(ManipulationError):
    pass


class PickRecoveryRequired(MotionFailed):
    """Pickup pode ser repetido após reposicionar a base."""

    def __init__(
        self,
        message: str,
        detected_pose,
        recovery_reason: int,
        moveit_error_code: int = 0,
    ) -> None:
        super().__init__(message)
        self.detected_pose = detected_pose
        self.recovery_reason = int(recovery_reason)
        self.moveit_error_code = int(moveit_error_code)


class ObjectOutOfReach(PickRecoveryRequired):
    pass


class StateConflict(ManipulationError):
    pass
