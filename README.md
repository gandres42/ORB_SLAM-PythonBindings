# ORB_SLAM3-PythonBindings
A python wrapper for ORB_SLAM3. Originally written for ORB_SLAM2
([https://github.com/raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)),
this copy has been ported to the
[gandres42/ORB_SLAM3](https://github.com/gandres42/ORB_SLAM3) fork of
ORB_SLAM3 v1.0. It builds against Python 3.14 / Boost 1.90 and OpenCV 4.

### Changes from the ORB_SLAM2 version

- **The module is now `orbslam3`** (`import orbslam3`).
- **No ORB_SLAM3 source patch is required.** The bindings build against the
  fork as installed by `make install`. ORB_SLAM3's `System` keeps its tracker
  and atlas private, so the bindings reach them through an explicit template
  instantiation (see the top of `src/ORBSlamPython.cpp`) to rebuild the full
  per-frame trajectory and to list keyframes.
- **ORB_SLAM3 features exposed:**
  - The inertial sensor modes `Sensor.IMU_MONOCULAR`, `Sensor.IMU_STEREO` and
    `Sensor.IMU_RGBD`. Every `process_image_*` / `load_and_process_*` call takes
    an optional `imu_measurements` argument: the IMU samples since the previous
    frame as an Nx7 array of `(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, t)`
    (m/s^2, rad/s, seconds).
  - The extra tracking states `TrackingState.RECENTLY_LOST` and
    `TrackingState.OK_KLT`.
- **Multi-map behaviour.** When ORB_SLAM3 loses track it starts a new map in its
  atlas, which has its own world frame. `get_trajectory_points()` and
  `get_keyframe_points()` only return poses from the active map, so they stay
  consistent with the map points. The trajectory therefore restarts after a new
  map is created, and the earlier poses come back once ORB_SLAM3 merges the
  maps.
- **Removed:** the agent-name constructor argument / `set_agent_name()` and
  `get_high_quality_mappoints()` / `pop_new_high_quality_mappoints()`. They
  belonged to the multi-agent ORB_SLAM2 fork and have no ORB_SLAM3 counterpart.
- `process_image_*` returns `True` when the frame was tracked (tracking state
  `OK`). ORB_SLAM3 always returns a pose, even when tracking fails.

## Installation

### Prerequisites

- The ORB_SLAM3 fork built **and** installed (`./build.sh`, then
  `cd build && sudo make install`). This installs the headers, `libORB_SLAM3.so`,
  the bundled DBoW2 / g2o / Sophus, the vocabulary
  (`<prefix>/share/ORB_SLAM3/Vocabulary/ORBvoc.txt`) and a CMake package under
  an install prefix (default `/usr/local`).
- ORB_SLAM3's compilation dependencies (Pangolin, Eigen3, OpenCV 4).
- Boost, specifically its python component matching your Python (e.g. `python314`).
- NumPy development headers (images are passed as numpy arrays, auto-converted to
  `cv::Mat`).

### Compilation

```
mkdir build
cd build
cmake ..
make
sudo make install
```

- If ORB_SLAM3 was installed somewhere other than a default prefix, pass
  `-DORB_SLAM3_DIR=<prefix>/lib/cmake/ORB_SLAM3`.
- The bindings are compiled with `-march=native`, like ORB_SLAM3. Both must be
  built on the same machine so that Eigen's alignment matches.
- The Python version and the matching Boost.Python component are detected
  automatically. The module and its type stubs (`orbslam3.pyi`) are installed
  into the interpreter's platform library directory (on Debian/Ubuntu,
  `/usr/local/lib/python<X.Y>/dist-packages`).

Verify your installation by typing
```
python3
>>> import orbslam3
```
And there should be no errors.

### Settings files

ORB_SLAM3 reads both its own settings format (`File.version: "1.0"`, with
`Camera1.fx`, ...) and the older ORB_SLAM2 format (`Camera.fx`, ...). An
ORB_SLAM2-style file must also set `Camera.type: "PinHole"` (or
`"KannalaBrandt8"`), which ORB_SLAM2 did not need.

### Examples

ORB_SLAM's examples have been re-implemented in python in the examples folder.
Run them with the same parameters as the ORB_SLAM3 examples, i.e.:
```
python3 orbslam_mono_kitti.py [PATH_TO_ORBSLAM3]/Vocabulary/ORBvoc.txt [PATH_TO_ORBSLAM3]/Examples/Monocular/KITTI00-02.yaml [PATH_TO_KITTI]/sequences/00/
```

## License
This code is licensed under the BSD Simplified license, although it requires and links to ORB_SLAM3, which is available under the GPLv3 license

It uses pyboostcvconverter (https://github.com/Algomorph/pyboostcvconverter) by Gregory Kramida under the MIT licence (see pyboostcvconverter-LICENSE).
