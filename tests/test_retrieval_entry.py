import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import contextlib
import io
import signal
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]


class RetrievalEntryTests(unittest.TestCase):
    def setUp(self):
        path = ROOT/'scripts/run_retrieval.py'
        self.assertTrue(path.is_file(), 'provide the current single-task entry')
        spec = importlib.util.spec_from_file_location('retrieval_entry', path)
        self.entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.entry)

    def test_current_profile_is_shared_and_does_not_activate_old_slots(self):
        profile = json.loads((ROOT/'configs/current_sim_task.json').read_text())
        scene = json.loads((ROOT/'configs/dev_multiscene_paired.json').read_text())['scenes'][1]
        generic = self.entry.task_config(profile, scene, 'generic')
        ours = self.entry.task_config(profile, scene, 'ours')
        self.assertEqual(generic.pop('slots')[0]['method'], 'generic')
        self.assertEqual(ours.pop('slots')[0]['method'], 'ours')
        self.assertEqual(generic, ours)
        self.assertEqual(ours['scenes'], [scene])
        self.assertEqual(ours['handoff_stop'], 'screened_candidate')
        self.assertEqual(ours['support_anchor'], 'exact_winner')
        self.assertEqual(ours['operational_gating'], 'v1.4')
        self.assertEqual(ours['observation_windows'], 3)
        with self.assertRaises(ValueError):
            self.entry.task_config(dict(profile, status='FROZEN_FOR_FORMAL'), scene, 'ours')

    def test_command_enables_shared_execution_and_never_replays_pose(self):
        command = self.entry.task_command(Path('/sim'), Path('/rm'), Path('/task'), Path('/out'))
        for flag in ('--integrated-joint-velocity', '--full-robot-manipulation', '--execution-clearance'):
            self.assertIn(flag, command)
        self.assertNotIn('--ground-replay-from', command)
        self.assertNotIn('--ground-at-candidate', command)
        self.assertEqual(command[0], '/sim/scripts/with_p450_env.bash')
        self.assertEqual(command[command.index('--slot')+1], '1')

    def test_status_preserves_running_failure_and_missing_measurement(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary); (path/'attempt/data').mkdir(parents=True)
            (path/'attempt/data/events.jsonl').write_text(
                json.dumps(dict(state='A6_ENV_RESULT', confirmed_candidate_count=2, round=2))+'\n'+
                '{"state": "GROUND_APPROACH"}\n'+ '{"state":')
            result = self.entry.task_status(path)
            self.assertEqual(result['last_state'], 'GROUND_APPROACH')
            self.assertEqual(result['confirmed_candidates'], 2)
            self.assertEqual(result['completed_windows'], 2)
            self.assertIsNone(result['retrieval_success'])
            (path/'attempt/attempt.json').write_text(json.dumps(dict(
                finish_wall=100, status='VALID_TRIAL', retrieval_success=False, reason='planning failed')))
            result = self.entry.task_status(path)
            self.assertFalse(result['retrieval_success'])
            self.assertEqual(result['first_failure'], 'planning failed')
            self.assertEqual(result['run_status'], 'FINISHED')

    def test_scene_loading_is_explicit_not_candidate_or_outcome_selected(self):
        path = ROOT/'configs/dev_multiscene_paired.json'
        scene = self.entry.load_scene(path, 'paired-hard-02')
        self.assertEqual(scene['seed'], 1301813125)
        with self.assertRaises(ValueError):
            self.entry.load_scene(path, None)
        with self.assertRaises(ValueError):
            self.entry.load_scene(path, 'no-such-scene')

    def test_dry_run_main_never_writes_or_launches(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/'unused'
            with patch.object(self.entry.subprocess, 'Popen') as launch, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(self.entry.main(['run', '--dry-run', '--output-dir', str(output)]), 0)
            launch.assert_not_called()
            self.assertFalse(output.exists())

    def test_existing_output_is_rejected_before_external_actions(self):
        with tempfile.TemporaryDirectory() as temporary:
            with patch.object(self.entry.subprocess, 'Popen') as launch, contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(FileExistsError):
                    self.entry.main(['run', '--output-dir', temporary])
            launch.assert_not_called()

    def test_wrapper_failure_before_attempt_is_preserved_and_queryable(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/'task'
            with patch.object(self.entry.socket, 'socket') as socket_mock, \
                 patch.object(self.entry.subprocess, 'check_output', return_value='commit\n'), \
                 patch.object(self.entry.subprocess, 'Popen', return_value=SimpleNamespace(wait=lambda: 66)), \
                 contextlib.redirect_stdout(io.StringIO()):
                socket_mock.return_value.__enter__.return_value.connect_ex.return_value = 111
                self.assertEqual(self.entry.main(['run', '--output-dir', str(output)]), 66)
            result = self.entry.task_status(output)
            self.assertEqual(result['run_status'], 'FINISHED')
            self.assertEqual(result['launcher_exit_code'], 66)
            self.assertIn('launcher', result['first_failure'])
            self.assertIsNone(result['retrieval_success'])

    def test_interrupt_waits_for_owned_runner_cleanup_without_killing_it(self):
        self.assertTrue(hasattr(self.entry, 'wait_for_launcher'))
        from unittest.mock import Mock
        process = Mock()
        process.wait.side_effect = [KeyboardInterrupt(), 1]
        self.assertEqual(self.entry.wait_for_launcher(process), 130)
        process.send_signal.assert_called_once_with(signal.SIGINT)
        process.kill.assert_not_called()
        self.assertEqual(process.wait.call_count, 2)

    def test_live_partial_metadata_reports_missing_instead_of_crashing(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary); (output/'attempt').mkdir()
            (output/'attempt/attempt.json').write_text('{"status":')
            result = self.entry.task_status(output)
            self.assertEqual(result['run_status'], 'RECORD_INCOMPLETE')
            self.assertIsNone(result['retrieval_success'])
            self.assertTrue(result['record_errors'])
