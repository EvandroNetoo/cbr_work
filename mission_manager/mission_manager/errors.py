"""Expected mission-manager failures."""


class MissionError(RuntimeError):
    """Base class for errors that can be reported to an ExecuteMission goal."""


class ConfigurationError(MissionError):
    """A YAML file or ROS parameter is invalid."""


class StepFailed(MissionError):
    """A child action failed while executing a plan step."""


class MissionCanceled(MissionError):
    """The parent ExecuteMission goal was canceled."""


class StateConflict(MissionError):
    """A requested transition conflicts with the mission-owned world state."""


class TaskNotFound(StepFailed):
    """A flexible pick observed an empty source; choose another task."""
