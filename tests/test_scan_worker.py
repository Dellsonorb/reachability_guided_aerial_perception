"""Real CSV -> worker -> saved arrays and paired policy, without ROS or GT."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from a6_pilot.policy import decide_policy
from a6_pilot.worker import observe
from a6_scoring_diagnostics import check_snapshot
from environment_belief import EnvironmentBeliefMapper, EnvironmentGridSpec
from reachability_guided_aerial_perception import GraspTCP
from reachability_guided_nbv import Viewpoint
from reachability_guided_nbv.geometry import sensor_transform
from sim_active_perception.core import A5Config
from sim_active_perception.worker import save_initial
from test_a5_core import inputs, scan
from test_field import candidate


class FiniteWorkerTests(unittest.TestCase):
    def test_real_finite_schedule_is_shared_and_survives_file_boundary(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            current = Viewpoint((-4, 0, 1.5), 0)
            transform = sensor_transform(current)
            direction = (np.array([.05, .05, 0.]) - transform[:3, 3]) @ transform[:3, :3]
            az = np.degrees(np.arctan2(direction[1], direction[0]))
            zen = 90 - np.degrees(np.arctan2(direction[2], np.hypot(*direction[:2])))
            values = np.zeros((40000, 3))
            values[:10000, 1:] = [az, zen]
            # Other packets point upward; preserve their original packet IDs.
            values[10000:, 2] = 0
            path = folder / 'small-schedule.csv'
            np.savetxt(path, values, delimiter=',', header='time,azimuth,zenith', comments='')
            config = A5Config(grid_width_m=3, grid_height_m=3, xy_offsets_m=(0,),
                              scan_pattern_path=str(path), scan_window_s=.1)
            field, raw = inputs([candidate(candidate_id='first', x=.011, y=.019, margin=.3)])
            grid = EnvironmentGridSpec((-1.5, -1.5), 30, 30)
            belief = EnvironmentBeliefMapper(grid).snapshot()
            choices, rankings = zip(*(decide_policy(field, raw, belief, current,
                method=method, round_count=1, config=config) for method in ('ours', 'generic')))
            self.assertEqual(rankings[0].candidates, rankings[1].candidates)
            np.testing.assert_array_equal(rankings[0].observation_opportunity, rankings[1].observation_opportunity)
            self.assertEqual(float(rankings[0].observation_opportunity.max()), .5)
            self.assertGreater(rankings[0].best_task.task_gain, 0)
            initial = save_initial(GraspTCP('a5-test', 'map', (0, 0, .4), (0, 0, 0, 1)),
                                   raw, config, folder / 'initial')
            empty = scan(np.empty((0, 3)), 1)
            observation = folder / 'observation.npz'
            np.savez(observation, points_xyz=empty.points_xyz, frame_id=empty.frame_id,
                     stamp_s=empty.stamp_s, T_map_sensor=empty.T_map_sensor)
            results = []
            for method in ('ours', 'generic'):
                output = folder / method
                result = observe(dict(initial_file=initial['initial_file'], observations=[str(observation)],
                    uav_pose=[*current.position_xyz, current.yaw_rad], method=method, output_dir=str(output)))
                ranking = json.loads((output / 'ranking.json').read_text())
                self.assertEqual(result['acquisition']['window_packets'], 2)
                self.assertEqual(ranking['acquisition']['model'], 'finite_scan_v1')
                with np.load(output / 'fields.npz') as arrays:
                    self.assertTrue(check_snapshot(ranking, arrays)['gain_and_cost_identity_pass'])
                    self.assertFalse(arrays['a2_observation_count'].any())
                    results.append(arrays['observation_opportunity'].copy())
            np.testing.assert_array_equal(*results)


if __name__ == '__main__':
    unittest.main()
