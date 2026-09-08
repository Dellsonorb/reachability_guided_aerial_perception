"""Recover one exact frozen A1 winner per feasible cell without revalidation."""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from numbers import Real

import numpy as np

from reachability_guided_aerial_perception import candidate_relevance
from reachability_guided_aerial_perception.model import CellState, ManipulationInterestField


@dataclass(frozen=True)
class WinnerAnchor:
    """Original IK-valid pose identity; source_id remains the A1 flat cell ID."""

    source_id: int
    candidate_id: str
    evaluation_index: int
    x: float
    y: float
    yaw: float
    relevance: float


def reconstruct_winner_anchors(field, evaluated_candidates):
    """Match A1's gated relevance maximum and first original evaluation tie.

    Supplied evaluations must be the complete original sequence. Agreement
    tolerance (1e-12) is numeric roundoff only, never a geometry allowance.
    Outside-grid and manipulation-invalid candidates never become anchors.
    """
    if not isinstance(field, ManipulationInterestField):
        raise ValueError('field must be ManipulationInterestField')
    if (not isinstance(evaluated_candidates, Sequence)
            or isinstance(evaluated_candidates, (str, bytes))
            or len(evaluated_candidates) != field.coverage.evaluated_candidates):
        raise ValueError('evaluated_candidates must be the complete original evaluation sequence')
    for name in ('cell_state', 'relevance', 'best_yaw'):
        if np.asarray(getattr(field, name)).shape != field.grid.shape:
            raise ValueError(f'A1 {name} shape must match grid')
    winners = {}
    for index, candidate in enumerate(evaluated_candidates):
        if not isinstance(candidate, Mapping):
            raise ValueError('each evaluated candidate must be a mapping')
        pose = []
        for name in ('bunker_x', 'bunker_y', 'bunker_yaw'):
            value = candidate.get(name)
            try:
                finite = np.isfinite(float(value))
            except (TypeError, ValueError, OverflowError):
                finite = False
            if isinstance(value, bool) or not isinstance(value, Real) or not finite:
                raise ValueError(f'evaluated candidate {name} must be a finite real')
            pose.append(float(value))
        candidate_id = candidate.get('candidate_id')
        if not isinstance(candidate_id, str) or not candidate_id.strip():
            raise ValueError('evaluated candidate_id must be a nonblank string')
        x, y, yaw = pose
        relevance = candidate_relevance(candidate, field.config)
        cell = field.grid.cell_index(x, y)
        if cell is None or relevance <= 0:
            continue
        if cell not in winners or relevance > winners[cell].relevance:
            winners[cell] = WinnerAnchor(cell[0] * field.grid.width_cells + cell[1],
                                         candidate_id, index, x, y, yaw, float(relevance))
    eligible = set(map(tuple, np.argwhere(np.isin(field.cell_state, [CellState.HIGH, CellState.LOW]))))
    if set(winners) != eligible:
        raise ValueError('exact winner cell coverage must match A1 HIGH/LOW cells')
    for cell, anchor in winners.items():
        if (not np.isclose(anchor.relevance, field.relevance[cell], rtol=0, atol=1e-12)
                or not np.isclose(anchor.yaw, field.best_yaw[cell], rtol=0, atol=1e-12)):
            raise ValueError(f'exact winner relevance/yaw disagrees with A1 cell {cell}')
    return tuple(sorted(winners.values(), key=lambda anchor: anchor.evaluation_index))
