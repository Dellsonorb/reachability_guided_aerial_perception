#!/usr/bin/python3
"""Native TCPROS transport-only contrast; no ROS master, Gazebo or actuation.

The local deterministic handler is NOT MoveIt and proves no collision result.
Identical full-message requests exercise serialization/connection overhead only.
"""
import argparse
import json
from pathlib import Path
import statistics
import time

import rospy
from rospy.impl import tcpros, tcpros_base
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args()
    # Fixed local socket setting avoids an unrelated master parameter lookup.
    # Identical for both contrasts; native TCPROS serialization is unchanged.
    tcpros_base._use_tcp_keepalive = True
    tcpros.init_tcpros()
    server = rospy.Service('/clearance_transport_probe', GetStateValidity,
        lambda request: GetStateValidityResponse(valid=request.robot_state.joint_state.position[0] < .5))
    connections = []
    original_handle = server.handle
    def handle(transport, header):
        connections.append(1)
        return original_handle(transport, header)
    server.handle = handle
    request = GetStateValidityRequest()
    request.robot_state.joint_state.name = ['joint%d' % i for i in range(7)]
    request.robot_state.joint_state.position = [0.] * 7
    request.robot_state.attached_collision_objects = [AttachedCollisionObject(
        link_name='tcp', object=CollisionObject(id='diagnostic_payload'))]
    results = []
    try:
        for persistent in (False, True):
            proxy = rospy.ServiceProxy('/clearance_transport_probe', GetStateValidity, persistent=persistent)
            proxy._get_service_uri = lambda _request: server.uri  # Explicit local test server, no master.
            before, elapsed = len(connections), []
            for i in range(300):
                request.robot_state.joint_state.position[0] = float(i % 2)
                started = time.perf_counter()
                response = proxy(request)
                elapsed.append(time.perf_counter() - started)
                assert response.valid is (i % 2 == 0)
            proxy.close()
            results.append(dict(persistent=persistent, calls=len(elapsed),
                connections=len(connections)-before, mean_s=statistics.mean(elapsed),
                median_s=statistics.median(elapsed), maximum_s=max(elapsed),
                response_mismatches=0))
    finally:
        server.shutdown('bounded transport diagnostic complete')
    report = dict(scope='transport only; not MoveIt collision timing or correctness', results=results,
                  original_check_samples_wall_guard_s=60, no_gazebo_start=True)
    options.output.parent.mkdir(parents=True, exist_ok=True)
    with options.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
