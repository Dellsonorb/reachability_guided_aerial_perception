import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import run_retrieval

ROOT = Path(__file__).resolve().parents[1]


class EvaluationEntryTests(unittest.TestCase):
    def test_one_evaluation_slot_dry_run_uses_shared_entry_and_keeps_whole_manifest(self):
        original = (ROOT/'configs/paper1_eval.json').read_text()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'not-created'
            text = io.StringIO()
            with contextlib.redirect_stdout(text):
                code = run_retrieval.main(['evaluation','--config',str(ROOT/'configs/paper1_eval.json'),
                                          '--slot','97','--output-dir',str(output),'--dry-run'])
            self.assertEqual(code,0)
            self.assertFalse(output.exists())
            self.assertIn('--slot 97',text.getvalue())
            for flag in ('--integrated-joint-velocity','--full-robot-manipulation',
                         '--execution-clearance','--ground-dynamics'):
                self.assertIn(flag,text.getvalue())
            self.assertIn('FROZEN_FOR_EVALUATION',text.getvalue())
            self.assertIn('"slot": 212',text.getvalue())
        self.assertEqual((ROOT/'configs/paper1_eval.json').read_text(),original)

    def test_pilot_config_cannot_be_promoted_by_the_new_entry(self):
        with self.assertRaisesRegex(ValueError,'named evaluation cohort'):
            run_retrieval.main(['evaluation','--config',str(ROOT/'configs/a6_formal.json'),
                                '--slot','1','--output-dir','/unused-eval-output','--dry-run'])


if __name__ == '__main__':
    unittest.main()
