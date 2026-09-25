"""Supported scene-detector bits and container colors."""

from interfaces.action import AnalyzeScene
from interfaces.msg import ContainerStampedDetection

APRILTAGS = AnalyzeScene.Goal.APRILTAGS
TABLE_SURFACE = AnalyzeScene.Goal.TABLE_SURFACE
CONTAINERS_HSV = AnalyzeScene.Goal.CONTAINERS_HSV
RED = ContainerStampedDetection.RED
BLUE = ContainerStampedDetection.BLUE
COLOR_NAMES = {RED: 'red', BLUE: 'blue'}
DEBUG_COLORS = {RED: (30, 30, 255), BLUE: (255, 80, 20)}
