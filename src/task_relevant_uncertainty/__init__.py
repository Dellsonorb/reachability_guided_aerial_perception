"""A3 footprint-aware task-relevant observation deficit; no ROS or live RM4D."""

from .anchors import WinnerAnchor, reconstruct_winner_anchors
from .core import (
    PoseEnvironmentState, PoseSupport, SupportState,
    TaskRelevantUncertaintyField, build_task_uncertainty,
)
from .geometry import FootprintSpec

__all__ = [
    'FootprintSpec', 'PoseEnvironmentState', 'PoseSupport', 'SupportState',
    'TaskRelevantUncertaintyField', 'build_task_uncertainty',
    'WinnerAnchor', 'reconstruct_winner_anchors',
]
