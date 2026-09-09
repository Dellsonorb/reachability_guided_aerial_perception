import copy
import importlib
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pose(label):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="planning", stamp=label + "-stamp"),
        pose=SimpleNamespace(label=label))


def trajectory(names=("j1", "j2"), positions=(.3, .4)):
    point = SimpleNamespace(positions=list(positions))
    return SimpleNamespace(joint_trajectory=SimpleNamespace(
        joint_names=list(names), points=[point]))


class Request:
    def __init__(self):
        self.header = None
        self.start_state = SimpleNamespace(
            joint_state=SimpleNamespace(name=[], position=[]), is_diff=False)
        self.ik_request = SimpleNamespace(
            group_name=None, ik_link_name=None, pose_stamped=None,
            robot_state=None, avoid_collisions=False, timeout=None)


class Group:
    def __init__(self, plans=None, execute=True):
        self.plans = list(plans or [(True, trajectory())])
        self.execute_result = execute
        self.calls = []

    def get_active_joints(self):
        return ["j1", "j2"]

    def set_start_state_to_current_state(self):
        self.calls.append(("current",))

    def set_joint_value_target(self, target):
        self.calls.append(("joint_target", copy.deepcopy(target)))

    def plan(self):
        self.calls.append(("plan",))
        return self.plans.pop(0)

    def execute(self, candidate, wait=True):
        self.calls.append(("execute", candidate, wait))
        return self.execute_result

    def stop(self):
        self.calls.append(("stop",))

    def clear_pose_targets(self):
        self.calls.append(("clear",))


class RefinedPregraspTests(unittest.TestCase):
    def setUp(self):
        self.module = load_script("a5_manipulation")
        self.pregrasp, self.grasp = pose("pregrasp"), pose("grasp")
        self.ik_state = SimpleNamespace(joint_state=SimpleNamespace(
            name=["j1", "j2"], position=[1.1, 1.2]))
        self.reverse = trajectory(positions=(.7, .8))
        self.forward = SimpleNamespace(
            fraction=1., error_code=SimpleNamespace(val=1),
            solution=trajectory())
        self.reverse_response = SimpleNamespace(
            fraction=1., error_code=SimpleNamespace(val=1), solution=self.reverse)
        self.ik_response = SimpleNamespace(
            error_code=SimpleNamespace(val=1), solution=self.ik_state)
        self.group = Group(plans=[(True, trajectory()), (True, trajectory())])
        self.node = SimpleNamespace(
            _move_group=self.group, _move_group_name="manipulator",
            _end_effector_link="tcp", _planning_time=9.,
            _moveit_server_timeout=3., _rm4d_pregrasp_plan_attempts=2,
            _cartesian_eef_step=.004, _cartesian_min_fraction=.999,
            _robot_state_from_joint_feedback=lambda: "fresh-state",
            _continuation_from_plan=lambda candidate, grasp: self.forward,
            _verify_tcp_pose=lambda target, label: (target, label))
        self.ik_requests, self.cart_requests = [], []

        def service(name, _kind):
            if name == "/compute_ik":
                def call(request):
                    self.ik_requests.append(copy.deepcopy(request))
                    return self.ik_response
                return call
            def call(request):
                self.cart_requests.append(copy.deepcopy(request))
                return self.reverse_response
            return call

        self.ros = SimpleNamespace(
            Duration=lambda value: ("duration", value),
            wait_for_service=lambda *_args, **_kwargs: None,
            ServiceProxy=service, ServiceException=RuntimeError,
            ROSException=RuntimeError)
        self.types = SimpleNamespace(
            GetPositionIK=object, GetPositionIKRequest=Request,
            GetCartesianPath=object, GetCartesianPathRequest=Request)

    def run_helper(self):
        with patch.object(self.module, "_ros_interfaces", return_value=(self.ros, self.types)):
            return self.module.execute_refined_pregrasp(
                self.node, self.pregrasp, self.grasp, ValueError)

    def test_forwards_exact_values_without_mutating_inputs(self):
        before = copy.deepcopy((self.pregrasp, self.grasp))
        self.run_helper()
        ik = self.ik_requests[0].ik_request
        self.assertEqual(ik.pose_stamped, self.grasp)
        self.assertEqual(ik.robot_state, "fresh-state")
        self.assertTrue(ik.avoid_collisions)
        self.assertEqual(ik.timeout, ("duration", 9.))
        reverse = self.cart_requests[0]
        self.assertEqual(reverse.header, self.pregrasp.header)
        self.assertEqual(reverse.start_state, self.ik_state)
        self.assertEqual(reverse.waypoints, [self.pregrasp.pose])
        self.assertEqual(reverse.max_step, .004)
        self.assertEqual(reverse.jump_threshold, 0.)
        self.assertTrue(reverse.avoid_collisions)
        self.assertEqual((self.pregrasp, self.grasp), before)
        self.assertIs(self.node._ground_last_grasp, self.grasp)
        self.assertIn(("joint_target", {"j1": .7, "j2": .8}), self.group.calls)

    def test_all_validation_precedes_execution(self):
        self.run_helper()
        names = [call[0] for call in self.group.calls]
        self.assertEqual(names, ["current", "joint_target", "plan", "execute", "stop", "clear"])

    def assert_no_execution(self):
        with self.assertRaises(ValueError):
            self.run_helper()
        self.assertFalse(any(call[0] == "execute" for call in self.group.calls))

    def test_failed_ik_never_executes(self):
        self.ik_response.error_code.val = -31
        logged = []
        self.node._publish_status = lambda state, **values: logged.append(dict(state=state, **values))
        self.assert_no_execution()
        self.assertEqual([row['error_code'] for row in logged if row['stage'] == 'grasp_ik'], [-31, -31])

    def test_missing_reverse_response_remains_a_rejected_plan(self):
        self.reverse_response = None
        self.assert_no_execution()

    def test_each_retry_uses_fresh_joint_feedback(self):
        seeds = iter(["fresh-one", "fresh-two"])
        self.node._robot_state_from_joint_feedback = lambda: next(seeds)
        failed = SimpleNamespace(error_code=SimpleNamespace(val=-31), solution=None)
        responses = iter([failed, self.ik_response])
        original_proxy = self.ros.ServiceProxy

        def service(name, kind):
            if name == "/compute_ik":
                def call(request):
                    self.ik_requests.append(copy.deepcopy(request))
                    return next(responses)
                return call
            return original_proxy(name, kind)

        self.ros.ServiceProxy = service
        self.run_helper()
        self.assertEqual([request.ik_request.robot_state for request in self.ik_requests],
                         ["fresh-one", "fresh-two"])

    def test_partial_reverse_never_executes(self):
        self.reverse_response.fraction = .5
        self.assert_no_execution()

    def test_failed_planning_never_executes(self):
        self.group.plans = [(False, trajectory()), (False, trajectory())]
        self.assert_no_execution()

    def test_partial_forward_never_executes(self):
        self.forward.fraction = .5
        self.assert_no_execution()

    def test_execution_failure_is_not_retried(self):
        self.group.execute_result = False
        with self.assertRaisesRegex(ValueError, "execution"):
            self.run_helper()
        self.assertEqual(sum(call[0] == "execute" for call in self.group.calls), 1)
        self.assertEqual(len(self.ik_requests), 1)

    def test_public_ros_service_exception_becomes_demo_error(self):
        self.ros.ServiceProxy = lambda *_args: lambda _request: (_ for _ in ()).throw(RuntimeError("transport"))
        with self.assertRaisesRegex(ValueError, "service.*transport"):
            self.run_helper()

    def test_forward_continuation_service_exception_never_executes(self):
        self.node._continuation_from_plan = lambda *_args: (
            _ for _ in ()).throw(RuntimeError("forward transport"))
        with self.assertRaisesRegex(ValueError, "service.*forward transport"):
            self.run_helper()
        self.assertFalse(any(call[0] == "execute" for call in self.group.calls))

    def test_joint_feedback_ros_exception_becomes_demo_error_without_execution(self):
        self.node._robot_state_from_joint_feedback = lambda: (
            _ for _ in ()).throw(RuntimeError("feedback transport"))
        with self.assertRaisesRegex(ValueError, "service.*feedback transport"):
            self.run_helper()
        self.assertFalse(any(call[0] == "execute" for call in self.group.calls))


class AdapterOverrideTests(unittest.TestCase):
    @staticmethod
    def adapter_modules(manipulation=None):
        modules = {
            "numpy": SimpleNamespace(), "rospy": SimpleNamespace(Subscriber=lambda *_a, **_k: None),
            "tf2_ros": SimpleNamespace(), "sensor_msgs": SimpleNamespace(point_cloud2=None),
            "sensor_msgs.msg": SimpleNamespace(PointCloud2=object),
            "std_msgs.msg": SimpleNamespace(String=object),
            "a5_ros_support": SimpleNamespace(
                WorkerError=RuntimeError, capture_pose_settled=None, finite_xyz=None,
                fresh_scan_stamp=None, merge_cloud_chunks=None, normalized_frame=None,
                pose_settled=None, pose_xyzyaw=None, rigid_transform=None,
                run_worker_request=None, yaw_quaternion=None)}
        modules["a5_manipulation"] = manipulation or load_script("a5_manipulation")
        return modules

    def test_explicit_continuation_routes_to_existing_a5_helper(self):
        adapter = load_script("run_a5_sim")
        helper_calls = []
        helper = lambda *args: helper_calls.append(args) or "refined"
        manipulation = SimpleNamespace(execute_refined_pregrasp=helper)
        class Base:
            def _execute_pregrasp(self, *_args):
                raise AssertionError("must not delegate")
        fake = SimpleNamespace(DemoError=RuntimeError, AirGroundPickDemo=Base)
        with patch.dict(sys.modules, self.adapter_modules(manipulation)):
            cls = adapter.build_adapter_class(fake, SimpleNamespace())
        node = object.__new__(cls)
        node._a5_refined_grasp = None
        target, continuation = object(), object()
        self.assertEqual(node._execute_pregrasp(target, continuation), "refined")
        self.assertEqual(
            helper_calls, [(node, target, continuation, RuntimeError)])

    def test_pick_scopes_exact_transformed_grasp_and_always_clears_it(self):
        adapter = load_script("run_a5_sim")
        generated_calls, inherited_scopes = [], []
        exact = object()

        def generate(*args):
            generated_calls.append(args)
            return SimpleNamespace(grasp=exact)

        class Base:
            def _pick_and_lift(self, sensor_pose, target):
                inherited_scopes.append(self._a5_refined_grasp)
                raise RuntimeError("inherited stop")

        fake = SimpleNamespace(DemoError=RuntimeError, AirGroundPickDemo=Base,
                               generate_top_down_grasp=generate)
        with patch.dict(sys.modules, self.adapter_modules()):
            cls = adapter.build_adapter_class(fake, SimpleNamespace())
        node = object.__new__(cls)
        node._a5_refined_grasp = None
        node._target_size = "size"
        node._pregrasp_height = "pre"
        node._lift_height = "lift"
        node._finger_pad_lower_edge_offset = "finger"
        node._contact_overlap = "overlap"
        node._surface_clearance = "clearance"
        node._map_frame = "map"
        node._initialize_moveit = lambda: SimpleNamespace(get_planning_frame=lambda: "planning")
        stamp = object()
        sensor = SimpleNamespace(header=SimpleNamespace(stamp=stamp))
        message = object()
        transformed = object()
        pose_calls, transform_calls = [], []
        node._pose_message = lambda value, frame, at: (
            pose_calls.append((value, frame, at)) or message)
        node._transform_pose = lambda value, frame: (
            transform_calls.append((value, frame)) or transformed)

        with self.assertRaisesRegex(RuntimeError, "inherited stop"):
            node._pick_and_lift(sensor, "target")

        self.assertEqual(generated_calls, [("target", "size", "pre", "lift", "finger",
                                            "overlap", "clearance")])
        self.assertEqual(pose_calls, [(exact, "map", stamp)])
        self.assertEqual(transform_calls, [(message, "planning")])
        self.assertEqual(inherited_scopes, [transformed])
        self.assertIsNone(node._a5_refined_grasp)

    def test_full_robot_pick_cache_uses_shared_contact_configuration_grasp(self):
        adapter = load_script("run_a5_sim")
        exact = object()
        calls = []

        class Base:
            def _generate_ground_grasp(self, target):
                calls.append(("generate", target))
                return SimpleNamespace(grasp=exact)

            def _pick_and_lift(self, sensor_pose, target):
                calls.append(("execute", target, self._a5_refined_grasp))
                return "physical-result"

        fake = SimpleNamespace(DemoError=RuntimeError, AirGroundPickDemo=Base)
        with patch.dict(sys.modules, self.adapter_modules()):
            cls = adapter.build_adapter_class(fake, SimpleNamespace())
        node = object.__new__(cls)
        node._full_robot_manipulation = True
        node._map_frame = "map"
        node._initialize_moveit = lambda: SimpleNamespace(get_planning_frame=lambda: "planning")
        node._pose_message = lambda value, frame, stamp: (value, frame, stamp)
        node._transform_pose = lambda value, frame: (value, frame)
        sensor = SimpleNamespace(header=SimpleNamespace(stamp=12.3))
        result = node._pick_and_lift(sensor, "runtime-refined-target")
        self.assertEqual("physical-result", result)
        self.assertEqual([
            ("generate", "runtime-refined-target"),
            ("execute", "runtime-refined-target", ((exact, "map", 12.3), "planning")),
        ], calls)
        self.assertIsNone(node._a5_refined_grasp)

    def test_implicit_refined_pregrasp_routes_to_helper(self):
        adapter = load_script("run_a5_sim")
        helper_calls = []
        helper = lambda *args: helper_calls.append(args) or "refined"
        manipulation = SimpleNamespace(execute_refined_pregrasp=helper)
        class Base:
            def _execute_pregrasp(self, *_args):
                raise AssertionError("must not delegate")
        fake = SimpleNamespace(DemoError=RuntimeError, AirGroundPickDemo=Base)
        with patch.dict(sys.modules, self.adapter_modules(manipulation)):
            cls = adapter.build_adapter_class(fake, SimpleNamespace())
        node = object.__new__(cls)
        node._a5_refined_grasp = "exact-grasp"
        self.assertEqual(node._execute_pregrasp("pregrasp"), "refined")
        self.assertEqual(helper_calls, [(node, "pregrasp", "exact-grasp", RuntimeError)])

    def test_implicit_pregrasp_without_scope_delegates(self):
        adapter = load_script("run_a5_sim")
        calls = []
        class Base:
            def _execute_pregrasp(self, target, continuation=None):
                calls.append((target, continuation))
                return "base"
        fake = SimpleNamespace(DemoError=RuntimeError, AirGroundPickDemo=Base)
        with patch.dict(sys.modules, self.adapter_modules()):
            cls = adapter.build_adapter_class(fake, SimpleNamespace())
        node = object.__new__(cls)
        node._a5_refined_grasp = None
        self.assertEqual(node._execute_pregrasp("target"), "base")
        self.assertEqual(calls, [("target", None)])

    def test_clearance_mode_does_not_bypass_shared_whole_chain_sim_pregrasp(self):
        adapter = load_script('run_a5_sim')
        calls = []
        class Base:
            def _execute_pregrasp(self, target, continuation=None):
                calls.append((target, continuation))
                return 'whole-chain'
        fake = SimpleNamespace(DemoError=RuntimeError, AirGroundPickDemo=Base)
        def legacy(*args):
            raise AssertionError('legacy approach-only planner bypassed whole-chain screen')
        with patch.dict(sys.modules, self.adapter_modules(SimpleNamespace(execute_refined_pregrasp=legacy))):
            cls = adapter.build_adapter_class(fake, SimpleNamespace())
        node = object.__new__(cls)
        node._execution_clearance = True
        node._a5_refined_grasp = 'old-cached-grasp'
        self.assertEqual(node._execute_pregrasp('target'), 'whole-chain')
        self.assertEqual(calls, [('target', None)])


if __name__ == "__main__":
    unittest.main()
