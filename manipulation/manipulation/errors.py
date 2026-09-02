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


class ObjectOutOfReach(MotionFailed):
    pass


class StateConflict(ManipulationError):
    pass
