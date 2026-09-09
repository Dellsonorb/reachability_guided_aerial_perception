"""Shared bounded execution screening, separate from observation confirmation."""
import copy

MAX_EXECUTION_CANDIDATES = 4
EXECUTION_VERSION = 'chassis-clearance-v1'


def select_execution_candidate(assessments, preview, publish):
    """Keep existing exact-winner relevance/tie order, reject plans before motion.

    Interface failures propagate; a failed planning attempt does not assert
    physical infeasibility of every branch. No actual execution is retried here.
    """
    confirmed = [c for c in assessments if c.get('confirmed') is True]
    ordered = sorted(confirmed, key=lambda c: -c['relevance'])[:MAX_EXECUTION_CANDIDATES]
    for index, candidate in enumerate(ordered):
        result = preview(copy.deepcopy(candidate))
        publish('GROUND_EXECUTION_SCREEN', candidate_id=candidate['candidate_id'],
                source_id=candidate.get('source_id'), rank=index+1,
                candidate_limit=MAX_EXECUTION_CANDIDATES, version=EXECUTION_VERSION, **result)
        if result.get('feasible') is True:
            chosen = copy.deepcopy(candidate)
            chosen['execution_screen'] = dict(result, version=EXECUTION_VERSION)
            return chosen
    return None
