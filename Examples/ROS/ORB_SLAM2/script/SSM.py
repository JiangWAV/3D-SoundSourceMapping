#!/usr/bin/env python2
import threading
import os

import numpy as np
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from ORB_SLAM2.msg import Pcm_Msg
from std_msgs.msg import String

def doa_from_angles(azimuth, elevation, degrees=False):
    if degrees:
        azimuth = azimuth / 180.0 * np.pi
        elevation = elevation / 180.0 * np.pi

    ce = np.cos(elevation)
    d = np.stack([
        ce * np.cos(azimuth),
        ce * np.sin(azimuth),
        np.sin(elevation)
    ], axis=-1)
    norm = np.linalg.norm(d, axis=-1, keepdims=True)
    return d / np.clip(norm, 1e-12, None)

def rotation_matrix(theta, matrix_type="Not Trans"):
    theta_x = float(theta[0] * np.pi / 180.0)
    theta_y = float(theta[1] * np.pi / 180.0)
    theta_z = float(theta[2] * np.pi / 180.0)

    sx, cx = np.sin(theta_x), np.cos(theta_x)
    sy, cy = np.sin(theta_y), np.cos(theta_y)
    sz, cz = np.sin(theta_z), np.cos(theta_z)

    r_x = np.array([[1.0, 0.0, 0.0],
                    [0.0, cx, -sx],
                    [0.0, sx, cx]], dtype=np.float64)
    r_y = np.array([[cy, 0.0, sy],
                    [0.0, 1.0, 0.0],
                    [-sy, 0.0, cy]], dtype=np.float64)
    r_z = np.array([[cz, -sz, 0.0],
                    [sz, cz, 0.0],
                    [0.0, 0.0, 1.0]], dtype=np.float64)

    if matrix_type == "Trans":
        return r_x.T.dot(r_y.T).dot(r_z.T)
    return r_z.dot(r_y).dot(r_x)

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    r = np.zeros((3, 3), dtype=np.float64)
    r[0, 0] = 1 - 2 * qy * qy - 2 * qz * qz
    r[0, 1] = 2 * qx * qy - 2 * qz * qw
    r[0, 2] = 2 * qx * qz + 2 * qy * qw
    r[1, 0] = 2 * qx * qy + 2 * qz * qw
    r[1, 1] = 1 - 2 * qx * qx - 2 * qz * qz
    r[1, 2] = 2 * qy * qz - 2 * qx * qw
    r[2, 0] = 2 * qx * qz - 2 * qy * qw
    r[2, 1] = 2 * qy * qz + 2 * qx * qw
    r[2, 2] = 1 - 2 * qx * qx - 2 * qy * qy
    return r

def load_doa_vectors(doa_file, degrees=True, azimuth_bias=0.0, elevation_bias=0.0):
    doa = np.load(doa_file)

    if doa.ndim == 2 and doa.shape[1] == 2:
        angles = doa.astype(np.float64).copy()
        angles[:, 0] += azimuth_bias
        angles[:, 1] += elevation_bias
        vectors = doa_from_angles(angles[:, 0], angles[:, 1], degrees=degrees)
        return vectors.reshape((-1, 1, 3))

    if doa.ndim == 2 and doa.shape[1] == 3:
        vectors = doa.astype(np.float64).reshape((-1, 1, 3))
    elif doa.ndim == 3 and doa.shape[2] == 3:
        vectors = doa.astype(np.float64)
    else:
        raise ValueError("Unsupported DOA shape %s in %s" % (str(doa.shape), doa_file))

    norm = np.linalg.norm(vectors, axis=2, keepdims=True)
    return vectors / np.clip(norm, 1e-12, None)

def skew(v):
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=np.float64)

def SO3_R(xi):
    omega = np.array(xi[:3], dtype=np.float64)
    theta = np.linalg.norm(omega)
    if theta < 1e-8:
        return np.eye(3)

    omega_hat = skew(omega)
    return (np.eye(3) +
            (np.sin(theta) / theta) * omega_hat +
            ((1 - np.cos(theta)) / (theta ** 2)) * omega_hat.dot(omega_hat))

def log_SO3(r):
    theta = np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    if theta < 1e-8:
        return np.zeros(3, dtype=np.float64)
    lnr = (theta / (2.0 * np.sin(theta))) * (r - r.T)
    return np.array([lnr[2, 1], lnr[0, 2], lnr[1, 0]], dtype=np.float64)

def camera_pose_to_matrix(pose):
    t = pose[:3].astype(np.float64)
    r = quaternion_to_rotation_matrix(pose[3], pose[4], pose[5], pose[6])
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = r
    T[:3, 3] = t
    return T

def camera_pose_to_measurements(camera_pose, use_relative_trajectory=True):
    transforms = []
    for pose in camera_pose:
        transforms.append(camera_pose_to_matrix(pose))

    origin_T = transforms[0].copy()
    if use_relative_trajectory:
        origin_inv = np.linalg.inv(origin_T)
    else:
        origin_inv = np.eye(4, dtype=np.float64)
        origin_T = np.eye(4, dtype=np.float64)

    measurements = np.zeros((len(transforms), 6), dtype=np.float64)
    for i, T_abs in enumerate(transforms):
        T = origin_inv.dot(T_abs)
        measurements[i, :3] = log_SO3(T[:3, :3])
        measurements[i, 3:] = T[:3, 3]

    return measurements, origin_T[:3, :3], origin_T[:3, 3]

def estimate_sources_with_mic_pose(pose_measurements, doa_vectors, mic_rotation_vec,
                                   mic_translation, min_observations=3):
    source_count = doa_vectors.shape[1]
    sources = np.zeros((source_count, 3), dtype=np.float64)
    mic_rotation = SO3_R(mic_rotation_vec)

    for source_id in range(source_count):
        a = np.zeros((3, 3), dtype=np.float64)
        b = np.zeros(3, dtype=np.float64)
        used = 0

        for i in range(len(pose_measurements)):
            r_camera = SO3_R(pose_measurements[i, :3])
            t_camera = pose_measurements[i, 3:]
            mic_position = t_camera + r_camera.dot(mic_translation)
            r_mic = r_camera.dot(mic_rotation)

            world_doa = r_mic.dot(doa_vectors[i, source_id])
            norm = np.linalg.norm(world_doa)
            if norm < 1e-12:
                continue
            world_doa = world_doa / norm

            projection = np.eye(3) - np.outer(world_doa, world_doa)
            a += projection
            b += projection.dot(mic_position)
            used += 1

        if used < min_observations:
            raise ValueError("Only %d valid DOA observations for source %d" %
                             (used, source_id))

        sources[source_id], _, _, _ = np.linalg.lstsq(a, b)

    return sources

def linearize_pose_landmark_constraint(mic_pose, sources, camera_pose, measurement):
    source_count = len(sources)
    e = np.zeros((3 * source_count, 1), dtype=np.float64)
    A = np.zeros((3 * source_count, 6), dtype=np.float64)
    B = np.zeros((3 * source_count, 3 * source_count), dtype=np.float64)

    camera_theta = camera_pose[:3]
    camera_t = camera_pose[3:]
    mic_theta = mic_pose[:3]
    mic_t = mic_pose[3:]

    r_camera = SO3_R(camera_theta)
    r_mic = r_camera.dot(SO3_R(mic_theta))
    r_mic_t = r_mic.T
    mic_position = camera_t + r_camera.dot(mic_t)

    for n in range(source_count):
        row = slice(3 * n, 3 * n + 3)
        v = sources[n] - mic_position
        v_norm = np.linalg.norm(v)
        if v_norm < 1e-12:
            raise ValueError("Source %d is too close to microphone position" % n)

        direction = v / v_norm
        predicted = r_mic_t.dot(direction)
        projection = np.eye(3) - np.outer(direction, direction)

        e[row] = (predicted - measurement[n]).reshape(3, 1)
        A[row, :3] = skew(predicted)
        A[row, 3:6] = -r_mic_t.dot(projection).dot(r_camera) / v_norm
        B[row, 3 * n:3 * n + 3] = r_mic_t.dot(projection) / v_norm

    return e, A, B

def optimize_unknown_mic_and_source(camera_pose, doa_vectors, initial_mic_rotation_vec,
                                    initial_mic_translation, initial_sources=None,
                                    start_index=0, min_observations=3,
                                    iterations=50, convergence_eps=1e-6,
                                    damping=1e-6, use_relative_trajectory=True,
                                    doa_std=1.0):
    start_index = max(0, int(start_index))
    n_obs = min(len(camera_pose), len(doa_vectors) - start_index)
    if n_obs < min_observations:
        raise ValueError("Need at least %d matched observations, got %d" %
                         (min_observations, n_obs))

    camera_pose = camera_pose[:n_obs]
    doa_vectors = doa_vectors[start_index:start_index + n_obs]
    pose_measurements, origin_R, origin_t = camera_pose_to_measurements(
        camera_pose, use_relative_trajectory=use_relative_trajectory)

    mic_pose = np.zeros(6, dtype=np.float64)
    mic_pose[:3] = np.array(initial_mic_rotation_vec, dtype=np.float64)
    mic_pose[3:] = np.array(initial_mic_translation, dtype=np.float64)

    if initial_sources is None:
        sources = estimate_sources_with_mic_pose(
            pose_measurements,
            doa_vectors,
            mic_pose[:3],
            mic_pose[3:],
            min_observations=min_observations)
    else:
        sources = np.array(initial_sources, dtype=np.float64).reshape((-1, 3))

    source_count = doa_vectors.shape[1]
    if len(sources) != source_count:
        raise ValueError("initial_sources count does not match DOA source count")

    state = np.zeros(6 + 3 * source_count, dtype=np.float64)
    state[:6] = mic_pose
    state[6:] = sources.reshape(-1)

    if np.isscalar(doa_std):
        std_vec = np.ones(3 * source_count, dtype=np.float64) * float(doa_std)
    else:
        std_vec = np.array(doa_std, dtype=np.float64).reshape(-1)
        if len(std_vec) == 3:
            std_vec = np.tile(std_vec, source_count)
    if len(std_vec) != 3 * source_count:
        raise ValueError("doa_std must be scalar, length 3, or length 3 * source_count")
    weight = np.diag(1.0 / np.clip(std_vec, 1e-12, None) ** 2)

    loss = 0.0
    last_update_norm = 0.0
    completed_iterations = 0
    for iteration in range(int(iterations)):
        H = np.zeros((len(state), len(state)), dtype=np.float64)
        b = np.zeros((len(state), 1), dtype=np.float64)
        loss = 0.0

        mic_pose = state[:6]
        sources = state[6:].reshape((source_count, 3))

        for i in range(n_obs):
            e, A, B = linearize_pose_landmark_constraint(
                mic_pose, sources, pose_measurements[i], doa_vectors[i])
            J = np.hstack((A, B))
            H += J.T.dot(weight).dot(J)
            b += J.T.dot(weight).dot(e)
            loss += float(e.T.dot(weight).dot(e))

        H += np.eye(len(state), dtype=np.float64) * float(damping)

        try:
            dx = np.linalg.solve(H, -b).reshape(-1)
        except np.linalg.LinAlgError:
            dx, _, _, _ = np.linalg.lstsq(H, -b)
            dx = dx.reshape(-1)

        state[:3] = log_SO3(SO3_R(state[:3]).dot(SO3_R(dx[:3])))
        state[3:] += dx[3:]

        last_update_norm = np.linalg.norm(dx)
        completed_iterations = iteration + 1
        if last_update_norm < convergence_eps:
            break

    sources = state[6:].reshape((source_count, 3))
    sources_map = np.zeros_like(sources)
    for i in range(source_count):
        sources_map[i] = origin_R.dot(sources[i]) + origin_t

    return {
        "mic_rotation_vec": state[:3].copy(),
        "mic_translation": state[3:6].copy(),
        "sources": sources,
        "sources_map": sources_map,
        "used_observations": n_obs * source_count,
        "iterations": completed_iterations,
        "last_update_norm": last_update_norm,
        "loss": loss
    }

def parse_initial_sources(value):
    if value is None or value == "":
        return None
    initial = np.array(value, dtype=np.float64)
    if initial.size == 0:
        return None
    return initial.reshape((-1, 3))


class visual_audio_extract:
    def __init__(self):
        self.xs = []
        self.ys = []
        self.zs = []
        self.quaternion = []
        self.lock = threading.Lock()
        self.system_start = False

        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_doa_file = os.path.abspath(os.path.join(
            script_dir, "..", "audio", "DOA.npy"))

        self.doa_file = rospy.get_param("~doa_file", default_doa_file)

        print(self.doa_file)
        self.trajectory_file = rospy.get_param("~trajectory_file", "")
        self.save_trajectory_file = rospy.get_param("~save_trajectory_file", "")
        self.audio_stride = int(rospy.get_param("~audio_stride", 3))
        self.doa_degrees = bool(rospy.get_param("~doa_degrees", True))
        self.doa_start_index = int(rospy.get_param("~doa_start_index", 0))
        self.azimuth_bias = float(rospy.get_param("~azimuth_bias", 0.0))
        self.elevation_bias = float(rospy.get_param("~elevation_bias", 0.0))
        self.min_observations = int(rospy.get_param("~min_observations", 3))
        self.opt_iterations = int(rospy.get_param("~opt_iterations", 50))
        self.convergence_eps = float(rospy.get_param("~convergence_eps", 1e-6))
        self.damping = float(rospy.get_param("~damping", 1e-6))
        self.doa_std = rospy.get_param("~doa_std", 1.0)
        self.use_relative_trajectory = bool(rospy.get_param("~use_relative_trajectory", True))
        self.initial_sources = parse_initial_sources(
            rospy.get_param("~initial_source_position", []))

        # These are initial guesses. The optimizer estimates the camera-to-mic pose.
        mic_rotation_deg = rospy.get_param("~mic_rotation_deg", [0.0, 90.0, -90.0])
        mic_translation = rospy.get_param("~mic_translation", [0.0, -0.1, 0])
        initial_mic_rotation = rotation_matrix(
            np.array(mic_rotation_deg, dtype=np.float64), matrix_type="Not Trans")
        self.initial_mic_rotation_vec = log_SO3(initial_mic_rotation)
        self.initial_mic_translation = np.array(mic_translation, dtype=np.float64)

        rospy.Subscriber("/camera_pose", PoseStamped, self.callback)
        rospy.Subscriber("/audio_stream", Pcm_Msg, self.audio_cb)
        rospy.Subscriber("/record_control", String, self.system_cb)
        self.SSP_pub = rospy.Publisher("/sound_source_position", Point, queue_size=1)
        self.point_msg = Point()

        self.audio_cb_count = 0
        self.camera_pose_audio = np.zeros((0, 7), dtype=np.float64)

    def callback(self, msg):
        with self.lock:
            self.xs.append(msg.pose.position.x)
            self.ys.append(msg.pose.position.y)
            self.zs.append(msg.pose.position.z)
            self.quaternion.append(np.array([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ], dtype=np.float64))
    
    def audio_cb(self, msg):
        if not self.system_start:
            return

        with self.lock:
            if len(self.xs) == 0 or len(self.quaternion) == 0:
                return

            self.audio_cb_count += 1
            if self.audio_cb_count < self.audio_stride:
                return

            current_camera_pose = np.array([self.xs[-1], self.ys[-1], self.zs[-1]],
                                           dtype=np.float64)
            current_camera_pose = np.append(current_camera_pose, self.quaternion[-1])
            self.camera_pose_audio = np.vstack((self.camera_pose_audio,
                                                current_camera_pose))
            self.audio_cb_count = 0

    def system_cb(self, msg):
        if msg.data == "start":
            self.system_start = True
            self.audio_cb_count = 0
            self.camera_pose_audio = np.zeros((0, 7), dtype=np.float64)
            rospy.loginfo("SSM recording started")
            self.publish_point(np.zeros(3, dtype=np.float64))
        elif msg.data == "stop":
            self.system_start = False
            rospy.loginfo("SSM recording stopped, poses recorded: %d",
                          len(self.camera_pose_audio))
            self.estimate_and_publish()

    def publish_point(self, point):
        self.point_msg.x = float(point[0])
        self.point_msg.y = float(point[1])
        self.point_msg.z = float(point[2])
        self.SSP_pub.publish(self.point_msg)

    def get_camera_pose_for_estimation(self):
        if self.trajectory_file:
            if not os.path.exists(self.trajectory_file):
                raise IOError("trajectory_file does not exist: %s" %
                              self.trajectory_file)
            return np.load(self.trajectory_file)

        if self.save_trajectory_file:
            np.save(self.save_trajectory_file, self.camera_pose_audio)
            rospy.loginfo("Saved SSM trajectory to %s", self.save_trajectory_file)

        return self.camera_pose_audio

    def estimate_and_publish(self):
        try:
            camera_pose = self.get_camera_pose_for_estimation()
            doa_vectors = load_doa_vectors(
                self.doa_file,
                degrees=self.doa_degrees,
                azimuth_bias=self.azimuth_bias,
                elevation_bias=self.elevation_bias)

            result = optimize_unknown_mic_and_source(
                camera_pose,
                doa_vectors,
                self.initial_mic_rotation_vec,
                self.initial_mic_translation,
                initial_sources=self.initial_sources,
                start_index=self.doa_start_index,
                min_observations=self.min_observations,
                iterations=self.opt_iterations,
                convergence_eps=self.convergence_eps,
                damping=self.damping,
                use_relative_trajectory=self.use_relative_trajectory,
                doa_std=self.doa_std)

            source = result["sources_map"][0]
            mic_r = result["mic_rotation_vec"]
            mic_t = result["mic_translation"]
            rospy.loginfo(
                "Estimated mic pose camera->mic rotvec: [%.4f, %.4f, %.4f], "
                "translation: [%.4f, %.4f, %.4f]",
                mic_r[0], mic_r[1], mic_r[2], mic_t[0], mic_t[1], mic_t[2])
            rospy.loginfo(
                "Estimated sound source from %d observations after %d iterations: "
                "[%.4f, %.4f, %.4f], update %.6f",
                result["used_observations"], result["iterations"],
                source[0], source[1], source[2], result["last_update_norm"])
            self.publish_point(source)
        except Exception as exc:
            rospy.logerr("Failed to estimate sound source position: %s", exc)

if __name__ == "__main__":
    rospy.init_node("sound_source_mapping", anonymous=True)
    visual_audio_extract()
    rospy.spin()
