import json
import tempfile
import unittest
from dataclasses import FrozenInstanceError, replace
from pathlib import Path

import numpy as np

from environment_belief import BeliefConfig, EnvironmentBeliefMapper, EnvironmentGridSpec, EnvironmentState
from reachability_guided_nbv.geometry import predict_visibility
from reachability_guided_nbv.model import NBVConfig, SensorModel, Viewpoint
from reachability_guided_nbv.finite_scan import FiniteScan


IDENTITY_SENSOR = SensorModel(T_uav_lidar=np.eye(4))


def unit(rows):
    values = np.asarray(rows, dtype=float)
    return values / np.linalg.norm(values, axis=1)[:, None]


def belief(grid=None, config=BeliefConfig(), occupied=()):
    grid = grid or EnvironmentGridSpec((0, 0), 2, 1, resolution_m=1)
    result = EnvironmentBeliefMapper(grid, config).snapshot()
    state = result.state.copy()
    for row, col in occupied:
        state[row, col] = EnvironmentState.OCCUPIED
    return replace(result, state=state)


def scan(directions, **kwargs):
    return FiniteScan(unit(directions), packet_rows=1, packet_period_s=.1,
                      sensor=IDENTITY_SENSOR, **kwargs)


def minimal_sdf(samples=4, period=.1, downsample=1):
    return f'''<sdf><model><link><sensor type="ray"><update_rate>{1 / period}</update_rate>
    <plugin filename="liblivox_laser_gazebo_plugins.so"><samples>{samples}</samples>
    <downsample>{downsample}</downsample><ray><scan>
    <horizontal><samples>3</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>2</max_angle></horizontal>
    <vertical><samples>3</samples><resolution>1</resolution><min_angle>-1</min_angle><max_angle>1</max_angle></vertical>
    </scan></ray></plugin></sensor></link></model></sdf>'''


class FiniteScanTests(unittest.TestCase):
    def test_consecutive_cyclic_windows_preserve_packet_identity(self):
        model = scan([[.25, .25, -1], [1.25, .25, -1], [1.25, .25, -1], [0, 0, 1]], window_s=.1)
        prediction = model.predict(belief(), Viewpoint((0, 0, 1), 0))
        np.testing.assert_array_equal(prediction.packet_hits[:, 0], [[1, 0], [0, 1], [0, 1], [0, 0]])
        np.testing.assert_array_equal(prediction.opportunity, [[.5, .75]])
        np.testing.assert_array_equal(prediction.unoccluded_opportunity, prediction.opportunity)
        self.assertEqual(prediction.status, 'VALID')

    def test_window_horizon_follows_first_last_packet_span_and_saturates(self):
        for window, expected in [(0, 1), (.1, 2), (.11, 3), (5, 51)]:
            with self.subTest(window=window):
                model = scan([[0, 0, -1]], window_s=window)
                self.assertEqual(model.metadata['window_packets'], expected)
                self.assertEqual(model.predict(belief(), Viewpoint((.25, .25, 1), 0)).opportunity[0, 0], 1)
        model = scan([[.25, .25, -1], [1.25, .25, -1], [0, 0, 1], [0, 0, 1]], window_s=100)
        np.testing.assert_array_equal(model.predict(belief(), Viewpoint((0, 0, 1), 0)).opportunity, [[1, 1]])

    def test_phase_fraction_matches_enumerated_windows_without_independence(self):
        directions = [[.2, .2, -1], [0, 0, 1], [1.2, .2, -1], [.3, .3, -1], [1.4, .3, -1], [0, 0, 1]]
        for packets in range(1, 9):
            model = FiniteScan(unit(directions), packet_rows=2, sensor=IDENTITY_SENSOR,
                               packet_period_s=1, window_s=packets - 1)
            result = model.predict(belief(), Viewpoint((0, 0, 1), 0))
            brute = np.mean([np.any(result.packet_hits[(start + np.arange(packets)) % 3], axis=0)
                             for start in range(3)], axis=0)
            np.testing.assert_array_equal(result.opportunity, brute)

    def test_mount_translation_pitch_and_uav_yaw_transform_actual_rays(self):
        sensor = SensorModel()
        direction = unit([[1, 0, -.2]])
        pose = Viewpoint((.3, -.2, 1.4), np.pi / 2)
        from reachability_guided_nbv.geometry import sensor_transform
        transform = sensor_transform(pose, sensor)
        ray = transform[:3, :3] @ direction[0]
        endpoint = transform[:3, 3] - transform[2, 3] / ray[2] * ray
        grid = EnvironmentGridSpec(tuple(endpoint[:2] - .025), 1, 1, resolution_m=.05)
        result = FiniteScan(direction, packet_rows=1, sensor=sensor).predict(belief(grid), pose)
        self.assertEqual(result.opportunity[0, 0], 1)

    def test_half_open_grid_internal_and_outer_boundaries_match_mapper(self):
        model = scan([[0, 0, -1]], window_s=0)
        for x, expected in [(0, [1, 0]), (1, [0, 1]), (2, [0, 0]), (-1e-8, [0, 0])]:
            with self.subTest(x=x):
                np.testing.assert_array_equal(model.predict(belief(), Viewpoint((x, .25, 1), 0)).opportunity[0], expected)

    def test_range_limits_are_strict_at_actual_plane_endpoint(self):
        model = scan([[0, 0, -1]], window_s=0)
        for config, expected in [(BeliefConfig(min_range_m=1), 0), (BeliefConfig(max_range_m=1), 0),
                                 (BeliefConfig(min_range_m=.999, max_range_m=1.001), 1)]:
            with self.subTest(config=config):
                self.assertEqual(model.predict(belief(config=config), Viewpoint((.2, .2, 1), 0)).opportunity[0, 0], expected)

    def test_no_downward_ray_and_nonzero_ground_height(self):
        pose = Viewpoint((.25, .25, 2), 0)
        self.assertFalse(scan([[0, 0, 1], [1, 0, 0]]).predict(belief(), pose).opportunity.any())
        result = scan([[1, 0, -1]]).predict(belief(config=BeliefConfig(ground_z_m=1)), pose)
        np.testing.assert_array_equal(result.opportunity, [[0, 1]])

    def test_invalid_sensor_origins_have_zero_outputs(self):
        model = scan([[0, 0, -1]])
        for pose, expected in [(Viewpoint((.2, .2, .5), 0), 'SENSOR_INSIDE_ASSUMED_PRISM'),
                               (Viewpoint((1.2, .2, 0), 0), 'SENSOR_AT_OR_BELOW_GROUND')]:
            result = model.predict(belief(occupied=[(0, 0)]), pose)
            self.assertEqual(result.status, expected)
            self.assertFalse(result.opportunity.any())
            self.assertFalse(result.unoccluded_opportunity.any())
            self.assertFalse(result.packet_hits.any())

    def test_actual_ray_through_prism_is_blocked_but_overflight_is_not(self):
        grid = EnvironmentGridSpec((-1, -1), 70, 20)
        model = scan([[4.05, .05, -1.5]], window_s=0)
        pose = Viewpoint((0, 0, 1.5), 0)
        through = model.predict(belief(grid, occupied=[(10, 40)]), pose)
        above = model.predict(belief(grid, occupied=[(10, 20)]), pose)
        self.assertEqual(through.unoccluded_opportunity[10, 50], 1)
        self.assertEqual(through.opportunity[10, 50], 0)
        self.assertEqual(above.opportunity[10, 50], 1)
        changed_height = model.predict(belief(grid, occupied=[(10, 20)]), pose,
                                       config=NBVConfig(assumed_height_m=2))
        self.assertEqual(changed_height.opportunity[10, 50], 0)

    def test_prism_boundary_contact_is_closed(self):
        grid = EnvironmentGridSpec((0, 0), 5, 2, resolution_m=1)
        model = scan([[4, 0, -2]], window_s=0)
        result = model.predict(belief(grid, occupied=[(0, 2)]), Viewpoint((0, 0, 2), 0))
        self.assertEqual(result.unoccluded_opportunity[0, 4], 1)
        self.assertEqual(result.opportunity[0, 4], 0)

    def test_endpoint_occlusion_does_not_reuse_cell_center(self):
        grid = EnvironmentGridSpec((0, 0), 5, 4, resolution_m=1)
        # Cell center (4.5,2.5) crosses occupied x[2,3],y[1,2].
        # Actual endpoint (4.05,2.95) passes above that cell in XY.
        state = belief(grid, occupied=[(1, 2)])
        sensor = SensorModel(T_uav_lidar=np.eye(4), min_elevation_deg=-89, max_elevation_deg=89)
        pose = Viewpoint((0, 1.5, 1.5), 0)
        self.assertFalse(predict_visibility(state, pose, sensor=sensor).visible[2, 4])
        model = FiniteScan(unit([[4.05, 1.45, -1.5]]), packet_rows=1, sensor=sensor)
        self.assertEqual(model.predict(state, pose).opportunity[2, 4], 1)

    def test_ray_cell_opportunity_does_not_apply_nominal_center_fov(self):
        model = scan([[.2, .2, -1]])
        pose = Viewpoint((0, 0, 1), 0)
        self.assertFalse(predict_visibility(belief(), pose, sensor=IDENTITY_SENSOR).visible[0, 0])
        self.assertEqual(model.predict(belief(), pose).opportunity[0, 0], 1)

    def test_occupied_destination_is_excluded_even_without_occlusion_ablation(self):
        result = scan([[0, 0, -1]]).predict(belief(occupied=[(0, 0)]), Viewpoint((.2, .2, 2), 0))
        self.assertEqual(result.opportunity[0, 0], 0)
        self.assertEqual(result.unoccluded_opportunity[0, 0], 0)

    def test_input_arrays_and_belief_are_unchanged_and_results_are_readonly(self):
        directions = unit([[0, 0, -1]])
        active = np.array([True])
        model = FiniteScan(directions, packet_rows=1, sensor=IDENTITY_SENSOR, emission_mask=active)
        directions[:] = [0, 0, 1]
        active[:] = False
        source = belief()
        before = {name: getattr(source, name).copy() for name in
                  ('state', 'occupied_evidence', 'free_evidence', 'observation_count', 'unknown_score')}
        result = model.predict(source, Viewpoint((.2, .2, 1), 0))
        self.assertEqual(result.opportunity[0, 0], 1)
        for name, original in before.items():
            np.testing.assert_array_equal(getattr(source, name), original)
        for array in (result.opportunity, result.unoccluded_opportunity, result.packet_hits, model.directions):
            self.assertFalse(array.flags.writeable)
        with self.assertRaises(FrozenInstanceError):
            model.window_s = 3
        json.dumps(model.metadata, allow_nan=False)

    def test_invalid_schedules_fail_before_prediction(self):
        for values in ([], [[0, 0, 0]], [[0, 0, -2]], [[0, 0, np.nan]], [[1, 2]]):
            with self.subTest(directions=values), self.assertRaises(ValueError):
                FiniteScan(values, packet_rows=1)
        for kwargs in ({'packet_rows': 0}, {'packet_rows': True}, {'packet_rows': 2},
                       {'packet_period_s': 0}, {'packet_period_s': np.inf}, {'window_s': -1},
                       {'window_s': np.nan}, {'emission_mask': [1]}, {'emission_mask': []}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                FiniteScan([[0, 0, -1]], **{'packet_rows': 1, **kwargs})


class CsvScheduleTests(unittest.TestCase):
    def test_csv_numeric_angles_and_packet_rows_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'directions.csv'
            path.write_text('Time/s,Azimuth/deg,Zenith/deg\n1,0,180\n2,90,90\n3,360,0\n4,180,90\n')
            model = FiniteScan.from_csv(path, packet_rows=2, sensor=IDENTITY_SENSOR)
            np.testing.assert_allclose(model.directions, [[0, 0, -1], [0, 1, 0], [0, 0, 1], [-1, 0, 0]], atol=1e-14)
            self.assertEqual(model.metadata['rows'], 4)
            self.assertEqual(model.metadata['packet_count'], 2)
            self.assertEqual(model.metadata['emitted_rows'], 4)
            self.assertEqual(model.metadata['scan_path'], str(path.resolve()))
            self.assertEqual(len(model.metadata['scan_sha256']), 64)

    def test_publisher_sdf_rounding_mask_keeps_original_packet_positions(self):
        with tempfile.TemporaryDirectory() as folder:
            csv, sdf = Path(folder) / 'scan.csv', Path(folder) / 'MID360.sdf'
            # vertical angular indices: 2.49 accepted,2.51 rejected; horizontal same.
            angles = [(0, 1.49), (0, 1.51), (2.49, 0), (2.51, 0)]
            csv.write_text('t,az,zen\n' + ''.join(f'{i},{np.degrees(a)},{90 + np.degrees(z)}\n'
                                                  for i, (a, z) in enumerate(angles)))
            sdf.write_text(minimal_sdf())
            model = FiniteScan.from_csv(csv, packet_rows=4, publisher_sdf_path=sdf)
            np.testing.assert_array_equal(model.emission_mask, [True, False, True, False])
            self.assertEqual(model.metadata['emitted_rows'], 2)
            self.assertEqual(model.metadata['publisher_sdf_path'], str(sdf.resolve()))
            self.assertEqual(model.metadata['packet_count'], 1)

    def test_publisher_downsampling_preserves_original_phase_blocks(self):
        with tempfile.TemporaryDirectory() as folder:
            csv, sdf = Path(folder) / 'scan.csv', Path(folder) / 'MID360.sdf'
            csv.write_text('t,az,zen\n' + ''.join(f'{i},0,90\n' for i in range(8)))
            sdf.write_text(minimal_sdf(downsample=2))
            model = FiniteScan.from_csv(csv, packet_rows=4, publisher_sdf_path=sdf)
            np.testing.assert_array_equal(model.emission_mask, [True, False] * 4)
            self.assertEqual(model.metadata['packet_count'], 2)

    def test_csv_and_publisher_mismatch_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            csv, sdf = Path(folder) / 'scan.csv', Path(folder) / 'MID360.sdf'
            for body in ('1,0,nan\n', '1,0\n', '1,0,90,2\n'):
                csv.write_text('t,az,zen\n' + body)
                with self.subTest(body=body), self.assertRaises(ValueError):
                    FiniteScan.from_csv(csv, packet_rows=1)
            csv.write_text('t,az,zen\n1,0,90\n2,0,90\n3,0,90\n4,0,90\n')
            for body in (minimal_sdf(samples=2), minimal_sdf(period=.2)):
                sdf.write_text(body)
                with self.subTest(body=body), self.assertRaises(ValueError):
                    FiniteScan.from_csv(csv, packet_rows=4, publisher_sdf_path=sdf)


if __name__ == '__main__':
    unittest.main()
