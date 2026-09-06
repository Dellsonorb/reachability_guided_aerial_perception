"""A4: one-step predicted observation-gain surrogate; no ROS or flight execution."""

from .model import NBVConfig, SensorModel, Viewpoint, generate_candidates
from .geometry import predict_visibility, sensor_transform
from .core import CandidateEvaluation, NBVResult, rank_viewpoints

__all__ = ['NBVConfig', 'SensorModel', 'Viewpoint', 'generate_candidates',
           'predict_visibility', 'sensor_transform', 'CandidateEvaluation', 'NBVResult', 'rank_viewpoints']
