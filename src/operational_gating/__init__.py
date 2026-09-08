"""Optional v1.1 evidence and footprint gates, independent of raw A2 state."""

from .core import (
    FootprintAssessment, OccupiedClass, OperationalEvidenceView, PerceivedTarget,
    assess_footprint, derive_operational_evidence,
)

__all__ = [
    'FootprintAssessment', 'OccupiedClass', 'OperationalEvidenceView', 'PerceivedTarget',
    'assess_footprint', 'derive_operational_evidence',
]
