#!/usr/bin/python3
"""A5-only refined-pregrasp planning through an approach-compatible IK branch."""

import math


def _ros_interfaces():
    """Keep ROS imports out of portable source-import and unit-test paths."""
    import rospy
    from moveit_msgs.srv import (
        GetCartesianPath, GetCartesianPathRequest,
        GetPositionIK, GetPositionIKRequest,
    )
    types = type("MoveItServices", (), dict(
        GetCartesianPath=GetCartesianPath,
        GetCartesianPathRequest=GetCartesianPathRequest,
        GetPositionIK=GetPositionIK,
        GetPositionIKRequest=GetPositionIKRequest))
    return rospy, types


def _complete_cartesian(response, minimum_fraction):
    return bool(
        response is not None and
        getattr(getattr(response, "error_code", None), "val", 1) == 1 and
        math.isfinite(response.fraction) and
        response.fraction >= minimum_fraction and
        response.solution.joint_trajectory.points)


def execute_refined_pregrasp(owner, target, grasp, demo_error):
    """Validate a grasp-seeded reverse branch before executing its pregrasp."""
    rospy, services = _ros_interfaces()
    exception_types = (rospy.ServiceException, rospy.ROSException)
    try:
        rospy.wait_for_service("/compute_ik", timeout=owner._moveit_server_timeout)
        rospy.wait_for_service(
            "/compute_cartesian_path", timeout=owner._moveit_server_timeout)
        compute_ik = rospy.ServiceProxy("/compute_ik", services.GetPositionIK)
        cartesian_path = rospy.ServiceProxy(
            "/compute_cartesian_path", services.GetCartesianPath)
    except exception_types as error:
        raise demo_error("MoveIt service unavailable: %s" % error)

    group = owner._move_group
    selected = None
    for _attempt in range(owner._rm4d_pregrasp_plan_attempts):
        ik_request = services.GetPositionIKRequest()
        ik_request.ik_request.group_name = owner._move_group_name
        ik_request.ik_request.ik_link_name = owner._end_effector_link
        ik_request.ik_request.pose_stamped = grasp
        try:
            ik_request.ik_request.robot_state = \
                owner._robot_state_from_joint_feedback()
        except exception_types as error:
            raise demo_error("MoveIt service failed: %s" % error)
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout = rospy.Duration(owner._planning_time)
        try:
            ik_response = compute_ik(ik_request)
        except exception_types as error:
            raise demo_error("MoveIt service failed: %s" % error)
        if getattr(getattr(ik_response, "error_code", None), "val", None) != 1:
            continue

        reverse_request = services.GetCartesianPathRequest()
        reverse_request.header = target.header
        reverse_request.start_state = ik_response.solution
        reverse_request.group_name = owner._move_group_name
        reverse_request.link_name = owner._end_effector_link
        reverse_request.waypoints = [target.pose]
        reverse_request.max_step = owner._cartesian_eef_step
        reverse_request.jump_threshold = 0.0
        reverse_request.avoid_collisions = True
        try:
            reverse = cartesian_path(reverse_request)
        except exception_types as error:
            raise demo_error("MoveIt service failed: %s" % error)
        if not _complete_cartesian(reverse, owner._cartesian_min_fraction):
            continue

        joint_trajectory = reverse.solution.joint_trajectory
        endpoint = dict(zip(
            joint_trajectory.joint_names,
            joint_trajectory.points[-1].positions))
        active = group.get_active_joints()
        if not all(name in endpoint and math.isfinite(endpoint[name])
                   for name in active):
            continue
        group.set_start_state_to_current_state()
        group.set_joint_value_target({name: endpoint[name] for name in active})
        planned = group.plan()
        success = bool(planned[0]) if isinstance(planned, tuple) else True
        candidate = planned[1] if isinstance(planned, tuple) else planned
        if (not success or not candidate.joint_trajectory.joint_names or
                not candidate.joint_trajectory.points):
            continue
        try:
            forward = owner._continuation_from_plan(candidate, grasp)
        except exception_types as error:
            raise demo_error("MoveIt service failed: %s" % error)
        if not _complete_cartesian(forward, owner._cartesian_min_fraction):
            continue
        selected = candidate
        break

    if selected is None:
        group.clear_pose_targets()
        raise demo_error("MoveIt refined pregrasp planning failed")
    if not group.execute(selected, wait=True):
        group.clear_pose_targets()
        raise demo_error("MoveIt refined pregrasp execution failed")
    group.stop()
    group.clear_pose_targets()
    return owner._verify_tcp_pose(target, "pregrasp")
