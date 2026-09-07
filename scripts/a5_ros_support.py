"""ROS-independent Python 3.8 helpers for the A5 simulator adapter."""

import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np


class WorkerError(RuntimeError):
    """The bounded core invocation or its response was invalid."""


def _vector(values, size):
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError("expected %d finite values" % size)
    return result


def finite_xyz(rows):
    """Keep measured returns only; invalid/no-return points are not free rays."""
    points = np.asarray(list(rows), dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("point cloud must have shape (N, 3)")
    return points[np.all(np.isfinite(points), axis=1) & np.any(points != 0., axis=1)]


def normalized_frame(frame):
    if not isinstance(frame, str) or not frame.lstrip("/"):
        raise ValueError("sensor frame must be nonempty")
    return frame.lstrip("/")


def rigid_transform(translation_xyz, quaternion_xyzw):
    """Full map <- sensor transform, including the LiDAR mount roll/pitch."""
    translation = _vector(translation_xyz, 3)
    quaternion = _vector(quaternion_xyzw, 4)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid transform quaternion")
    x, y, z, w = quaternion / norm
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    result[:3, 3] = translation
    return result


def yaw_quaternion(yaw):
    yaw = float(yaw)
    if not math.isfinite(yaw):
        raise ValueError("yaw must be finite")
    return (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))


def pose_xyzyaw(transform):
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("expected finite 4x4 transform")
    return [float(value) for value in transform[:3, 3]] + [
        math.atan2(transform[1, 0], transform[0, 0])]


def pose_settled(current, goal, velocity, position_tolerance, yaw_tolerance, speed_tolerance):
    try:
        current, goal = _vector(current, 4), _vector(goal, 4)
        velocity = _vector(velocity, 3)
        tolerances = _vector([position_tolerance, yaw_tolerance, speed_tolerance], 3)
    except (TypeError, ValueError):
        return False
    if np.any(tolerances < 0):
        return False
    yaw_error = math.atan2(math.sin(current[3] - goal[3]), math.cos(current[3] - goal[3]))
    return bool(np.linalg.norm(current[:3] - goal[:3]) <= position_tolerance
                and abs(yaw_error) <= yaw_tolerance
                and np.linalg.norm(velocity) <= speed_tolerance)


def capture_pose_settled(current, stamped_pose, goal, velocity,
                         position_tolerance, yaw_tolerance, speed_tolerance):
    """Accept slow common drift, but not a delayed cloud from another pose."""
    try:
        current = _vector(current, 4)
        goal = _vector(goal, 4)
    except (TypeError, ValueError):
        return False
    acquisition_goal = [current[0], current[1], current[2], goal[3]]
    return (pose_settled(current, acquisition_goal, velocity,
                         position_tolerance, yaw_tolerance, speed_tolerance)
            and pose_settled(stamped_pose, acquisition_goal, [0., 0., 0.],
                             position_tolerance, yaw_tolerance, speed_tolerance))


def fresh_scan_stamp(stamp_s, previous_stamp_s, capture_start_s):
    return bool(math.isfinite(stamp_s) and stamp_s > 0
                and stamp_s > previous_stamp_s and stamp_s > capture_start_s)


def merge_cloud_chunks(chunks):
    """Return one observation, anchored at the last chunk's sensor pose.

    Each chunk uses its own map transform. This changes coordinates between
    packets; it does not deskew points within a packet or create extra votes.
    """
    chunks = list(chunks)
    if not chunks:
        raise ValueError("cloud window must contain at least one chunk")
    frames = [normalized_frame(chunk["frame_id"]) for chunk in chunks]
    if any(frame != frames[0] for frame in frames):
        raise ValueError("cloud window sensor frames must match")
    stamps = np.asarray([chunk["stamp_s"] for chunk in chunks], dtype=np.float64)
    if (not np.all(np.isfinite(stamps)) or np.any(stamps <= 0)
            or np.any(np.diff(stamps) <= 0)):
        raise ValueError("cloud window stamps must be finite, positive and strictly increasing")
    matrices = np.asarray([chunk["T_map_sensor"] for chunk in chunks], dtype=np.float64)
    if matrices.shape != (len(chunks), 4, 4) or not np.all(np.isfinite(matrices)):
        raise ValueError("cloud chunks require finite 4x4 transforms")
    rotations = matrices[:, :3, :3]
    if (not np.allclose(matrices[:, 3, :], [0, 0, 0, 1], rtol=0, atol=1e-9)
            or not np.allclose(rotations.swapaxes(1, 2) @ rotations, np.eye(3), rtol=0, atol=1e-6)
            or not np.allclose(np.linalg.det(rotations), 1., rtol=0, atol=1e-6)):
        raise ValueError("cloud chunks require proper rigid transforms")
    reexpressed, counts = [], []
    for chunk, matrix in zip(chunks, matrices):
        points = np.asarray(chunk["points_xyz"], dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise ValueError("cloud chunks require finite (N, 3) points")
        last_from_chunk = np.linalg.solve(matrices[-1], matrix)
        reexpressed.append(points @ last_from_chunk[:3, :3].T + last_from_chunk[:3, 3])
        counts.append(len(points))
    return dict(points_xyz=np.concatenate(reexpressed), T_map_sensor=matrices[-1].copy(),
                stamp_s=np.asarray(stamps[-1]), frame_id=np.asarray(frames[-1]),
                chunk_stamps_s=stamps, chunk_point_counts=np.asarray(counts, dtype=np.int64),
                chunk_T_map_sensor=matrices)


def _response_numbers(values, size):
    if not isinstance(values, list) or not all(type(value) in (int, float) for value in values):
        raise ValueError("decision coordinates must be JSON numbers")
    return _vector(values, size)


def validate_worker_response(response, operation, expected_round=None):
    """Validate the boundary without rounding or reconstructing a candidate."""
    if not isinstance(response, dict) or response.get("ok") is not True:
        raise WorkerError("core failed: %s" % (
            response.get("error", "invalid response") if isinstance(response, dict) else "invalid response"))
    if operation == "init":
        if not isinstance(response.get("initial_file"), str) or not response["initial_file"]:
            raise WorkerError("core init response has no initial_file")
        return response
    if operation != "observe":
        raise WorkerError("unsupported core operation")
    round_number = response.get("round")
    if (type(round_number) is not int or round_number < 1
            or (expected_round is not None and round_number != expected_round)):
        raise WorkerError("core returned wrong observation round")
    if not all(key in response for key in ("stop_reason", "next_viewpoint", "selected_candidate")):
        raise WorkerError("core observation response is incomplete")
    stop = response["stop_reason"]
    if stop is not None and (not isinstance(stop, str) or not stop):
        raise WorkerError("invalid stop reason")
    try:
        if stop is None:
            _response_numbers(response["next_viewpoint"], 4)
        elif response["next_viewpoint"] is not None:
            raise ValueError("stopped response still has a next viewpoint")
        selected = response["selected_candidate"]
        if selected is not None:
            if (not isinstance(selected, dict)
                    or not isinstance(selected.get("candidate_id"), str)
                    or not selected["candidate_id"]
                    or type(selected.get("source_id")) is not int
                    or selected["source_id"] < 0):
                raise ValueError("candidate identity is missing")
            _response_numbers([selected[key] for key in ("x", "y", "yaw", "relevance")], 4)
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise WorkerError("invalid core decision: %s" % error) from error
    return response


def run_worker_request(core_python, worker_script, repo_src, request, output_dir, label, timeout):
    """Run one finite worker; retain request, raw response, and stdout/stderr."""
    if not math.isfinite(timeout) or timeout <= 0:
        raise WorkerError("core timeout must be positive")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    exchange = Path(tempfile.mkdtemp(prefix=label + "-", dir=str(output_dir)))
    request_path, response_path = exchange / "request.json", exchange / "response.json"
    log_path = exchange / "worker.log"
    try:
        request_path.write_text(json.dumps(request, indent=2, sort_keys=True, allow_nan=False) + "\n",
                                encoding="utf-8")
        environment = os.environ.copy()
        # Noetic's Python 3.8 package path must not leak into the Python 3.10 core.
        environment["PYTHONPATH"] = str(Path(repo_src).resolve())
        with log_path.open("w", encoding="utf-8") as log:
            completed = subprocess.run(
                [str(core_python), str(worker_script), "--request", str(request_path),
                 "--response", str(response_path)],
                stdout=log, stderr=subprocess.STDOUT, env=environment,
                timeout=timeout, check=False)
        if completed.returncode != 0:
            raise WorkerError("core exited %d; see %s" % (completed.returncode, log_path))
        response = json.loads(response_path.read_text(encoding="utf-8"))
        return validate_worker_response(response, request["op"],
                                        len(request["observations"]) if request["op"] == "observe" else None)
    except subprocess.TimeoutExpired as error:
        raise WorkerError("core timed out after %s s; see %s" % (timeout, log_path)) from error
    except (OSError, ValueError, KeyError) as error:
        raise WorkerError("core request/response failed: %s; see %s" % (error, exchange)) from error
