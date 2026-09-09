import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ExecutionSelectionTests(unittest.TestCase):
    def module(self):
        spec = importlib.util.spec_from_file_location('execution_selection', ROOT/'scripts/a5_execution_selection.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_shared_bounded_order_uses_only_confirmed_exact_winners(self):
        module = self.module()
        candidates = [dict(candidate_id=str(i), confirmed=i != 0, relevance=1., source_id=i,
                           x=float(i), y=0., yaw=0.) for i in range(7)]
        visited, events = [], []
        def check(candidate):
            visited.append(candidate['candidate_id'])
            return dict(feasible=candidate['candidate_id'] == '3', reason='chassis_guard')
        chosen = module.select_execution_candidate(candidates, check, lambda state, **kw: events.append((state, kw)))
        self.assertEqual(visited, ['1', '2', '3'])
        self.assertEqual(chosen['candidate_id'], '3')
        self.assertEqual(chosen['x'], 3.)
        self.assertNotIn('execution_screen', candidates[3])
        self.assertEqual(len(events), 3)

    def test_does_not_retry_beyond_four_or_pick_unconfirmed(self):
        module = self.module()
        candidates = [dict(candidate_id=str(i), confirmed=True, relevance=1.) for i in range(6)]
        visited = []
        result = module.select_execution_candidate(candidates,
                    lambda c: (visited.append(c['candidate_id']) or dict(feasible=False)), lambda *a, **k: None)
        self.assertIsNone(result)
        self.assertEqual(visited, ['0', '1', '2', '3'])

    def test_relevance_order_retains_first_tie_and_propagates_interface_error(self):
        module = self.module()
        candidates = [dict(candidate_id='low', confirmed=True, relevance=.2),
                      dict(candidate_id='first', confirmed=True, relevance=.8),
                      dict(candidate_id='second', confirmed=True, relevance=.8)]
        chosen = module.select_execution_candidate(candidates, lambda c: dict(feasible=True), lambda *a, **k: None)
        self.assertEqual(chosen['candidate_id'], 'first')
        def failed(_):
            raise RuntimeError('service unavailable')
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            module.select_execution_candidate(candidates, failed, lambda *a, **k: None)
