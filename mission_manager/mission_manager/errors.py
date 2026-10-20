"""Expected mission-manager failures."""


class MissionError(RuntimeError):
    """Base class for errors that can be reported to an ExecuteMission goal."""


class ConfigurationError(MissionError):
    """A YAML file or ROS parameter is invalid."""


class StepFailed(MissionError):
    """A child action failed while executing a plan step."""


class ActionUncertain(StepFailed):
    """A dispatched child action has no confirmed terminal result."""


class RetrieveFailed(StepFailed):
    """Cargo retrieval requires stopping the mission, never abandoning the cube."""


class MissionCanceled(MissionError):
    """The parent ExecuteMission goal was canceled."""


class StateConflict(MissionError):
    """A requested transition conflicts with the mission-owned world state."""


class TaskNotFound(StepFailed):
    """A flexible pick observed an empty source; choose another task."""


class DestinationUnavailable(StepFailed):
    """All reachable search points were exhausted without a usable destination."""


class PrecisionSlotOccupied(StepFailed):
    """The final PP snapshot blocked release while the object remains held."""


class PrecisionObjectAlreadyCorrect(MissionError):
    """A fresh PP observation made a pending pickup unnecessary."""


class PrecisionTaskFailed(StepFailed):
    """A PP object's bounded retries were exhausted, with known inventory."""

    def __init__(self, step, error):
        super().__init__(str(error))
        self.step = step
