"""Type stubs for the ``orbslam3`` Boost.Python extension module.

These are maintained by hand (Boost.Python does not emit usable signatures for
automatic stub generators) and installed alongside the compiled ``orbslam3.so``
so editors / type checkers can offer autocomplete and signature help.

Keep this in sync with the bindings registered in ``src/ORBSlamPython.cpp``.
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

class Sensor(int):
    """Input sensor mode passed to :class:`System`."""

    MONOCULAR: Sensor
    STEREO: Sensor
    RGBD: Sensor
    IMU_MONOCULAR: Sensor
    IMU_STEREO: Sensor
    IMU_RGBD: Sensor
    name: str
    names: dict[str, Sensor]
    values: dict[int, Sensor]

class TrackingState(int):
    """Tracker state reported by :meth:`System.get_tracking_state`."""

    SYSTEM_NOT_READY: TrackingState
    NO_IMAGES_YET: TrackingState
    NOT_INITIALIZED: TrackingState
    OK: TrackingState
    RECENTLY_LOST: TrackingState
    LOST: TrackingState
    OK_KLT: TrackingState
    name: str
    names: dict[str, TrackingState]
    values: dict[int, TrackingState]

# A keyframe / trajectory entry: (timestamp, then a row-major 3x4 camera-to-world
# pose) i.e. (t, r00, r01, r02, tx, r10, r11, r12, ty, r20, r21, r22, tz).
Pose = tuple[float, float, float, float, float, float, float, float, float, float, float, float, float]
# A map point in world coordinates: (x, y, z).
Point3 = tuple[float, float, float]
# A tracked feature: undistorted 2D pixel location plus its triangulated 3D
# world point, i.e. (u, v, x, y, z).
Feature = tuple[float, float, float, float, float]
# IMU measurements since the previous frame: an Nx7 array (or sequence of rows)
# of (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp), with the
# accelerometer in m/s^2, the gyroscope in rad/s and the timestamp in seconds.
# Only used by the IMU_* sensor modes.
ImuMeasurements = np.ndarray | Sequence[Sequence[float]] | None

class System:
    """Python wrapper around ``ORB_SLAM3::System``."""

    def __init__(
        self,
        vocab_file: str,
        settings_file: str,
        sensor: Sensor = ...,
    ) -> None: ...
    def initialize(self) -> bool:
        """Construct the underlying ORB_SLAM3 system. Call once before tracking."""
        ...
    def is_running(self) -> bool: ...
    def reset(self) -> None: ...
    def shutdown(self) -> None: ...

    # --- Tracking -----------------------------------------------------------
    # process_*: image(s) are passed as numpy arrays (HxW or HxWx3, uint8;
    # depth as uint16 / float32). Returns True if the frame was tracked
    # (tracking state OK).
    def process_image_mono(self, image: np.ndarray, timestamp: float, imu_measurements: ImuMeasurements = None) -> bool: ...
    def process_image_stereo(
        self, left_image: np.ndarray, right_image: np.ndarray, timestamp: float, imu_measurements: ImuMeasurements = None
    ) -> bool: ...
    def process_image_rgbd(
        self, image: np.ndarray, depth_image: np.ndarray, timestamp: float, imu_measurements: ImuMeasurements = None
    ) -> bool: ...
    # load_and_process_*: read the image(s) from disk, then track.
    def load_and_process_mono(self, image_file: str, timestamp: float, imu_measurements: ImuMeasurements = None) -> bool: ...
    def load_and_process_stereo(
        self, left_image_file: str, right_image_file: str, timestamp: float, imu_measurements: ImuMeasurements = None
    ) -> bool: ...
    def load_and_process_rgbd(
        self, image_file: str, depth_image_file: str, timestamp: float, imu_measurements: ImuMeasurements = None
    ) -> bool: ...

    # --- Outputs ------------------------------------------------------------
    def get_tracking_state(self) -> TrackingState: ...
    def get_num_features(self) -> int: ...
    def get_num_matched_features(self) -> int: ...
    def get_keyframe_points(self) -> list[Pose]:
        """Keyframe poses of the active map."""
        ...
    def get_trajectory_points(self) -> list[Pose]:
        """Full per-frame camera trajectory of the active map, including non-keyframe poses.

        Reconstructed from the tracker's poses relative to their reference
        keyframes, so it reflects bundle adjustment and loop closure. Poses are
        in the active map's world frame (the same frame as the map points).
        Frames that were not localized are skipped, as are frames anchored in
        another map: when ORB-SLAM3 loses track it starts a new map and the
        trajectory restarts until that map is merged back.
        """
        ...
    def get_tracked_mappoints(self) -> list[Point3]: ...
    def get_tracked_features(self) -> list[Feature]:
        """Currently tracked features: undistorted 2D pixel + triangulated 3D point.

        Index-aligned pairing of the current frame's keypoints and their
        associated map points; features without a triangulated landmark yet
        are omitted.
        """
        ...

    # --- Configuration ------------------------------------------------------
    def set_mode(self, mode: Sensor) -> None: ...
    def set_use_viewer(self, use_viewer: bool) -> None: ...
    def save_settings(self, settings: dict[str, Any]) -> bool: ...
    def load_settings(self) -> dict[str, Any]: ...
    @staticmethod
    def save_settings_file(settings: dict[str, Any], settings_filename: str) -> bool: ...
    @staticmethod
    def load_settings_file(settings_filename: str) -> dict[str, Any]: ...
