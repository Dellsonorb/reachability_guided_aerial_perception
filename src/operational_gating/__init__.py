"""Optional object-aware evidence and footprint gates, independent of raw A2."""

from .core import (
    FootprintAssessment, OccupiedClass, OperationalEvidenceView, PerceivedTarget,
    ambiguous_footprint_diagnostics, assess_footprint, derive_operational_evidence,
)
from .subcell import (
    AMBIGUOUS_ENDPOINT_PROFILE, AMBIGUOUS_ENDPOINT_RADIUS_M,
    AmbiguousEndpointEvidence, AmbiguousFootprintDiagnostics, disk_footprint_intersections,
)

__all__ = [
    'FootprintAssessment', 'OccupiedClass', 'OperationalEvidenceView', 'PerceivedTarget',
    'assess_footprint', 'derive_operational_evidence', 'ambiguous_footprint_diagnostics',
    'AMBIGUOUS_ENDPOINT_PROFILE', 'AMBIGUOUS_ENDPOINT_RADIUS_M',
    'AmbiguousEndpointEvidence', 'AmbiguousFootprintDiagnostics', 'disk_footprint_intersections',
]
