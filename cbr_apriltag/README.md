# CBR AprilTag

This package subscribes to a **rectified** `sensor_msgs/Image` and its matching
`sensor_msgs/CameraInfo`, detects AprilTags, and uses `tf2` to publish their
poses in `base_link`.

The camera driver publishes `camera_optical_frame` in the image header. The
URDF provides this frame from the physical `camera_link` mounted on link4.

## Run

Install the Python detector in the project virtualenv. The environment must
have access to the ROS Python packages, normally by being created with
`--system-site-packages`:

```bash
cd ~/ros2_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install pupil-apriltags
python -c "import rclpy, cv2, pupil_apriltags; import numpy; print(numpy.__version__)"
```

The detector converts the common ROS image encodings directly and does not
load `cv_bridge`; this avoids its NumPy 1.x binary ABI constraint in a LeRobot
environment using NumPy 2.

The launch uses the active virtualenv automatically and also searches for
`<workspace>/.venv/bin/python`. Override it with
`python_executable:=/path/to/.venv/bin/python` or the
`CBR_APRILTAG_PYTHON` environment variable.

Build and source the workspace, then launch:

```bash
colcon build --packages-select cbr_apriltag so_arm_101_description
source install/setup.bash
ros2 launch cbr_apriltag apriltag.launch.py \
  image_topic:=/camera/image_rect \
  camera_info_topic:=/camera/camera_info
```

The tag side is 32 mm by default. The preferred published interfaces are:

Detection is on-demand. While idle, the process and the initialized detector
stay in memory, but the image, `CameraInfo` and TF subscriptions are removed.
The node also calls `/camera/set_capture` to stop USB acquisition. A
goal sent to `/apriltags/analyze` re-enables capture and all required inputs;
the last completed goal returns them to the idle state after 0.5 s. A finite
goal processes for the requested duration; a zero duration runs continuously
until cancelled. Multiple goals share one detector pass per image.

```bash
ros2 action send_goal /apriltags/analyze \
  cbr_interfaces/action/AnalyzeAprilTags \
  "{duration: {sec: 5, nanosec: 0}}" --feedback
```

The result contains the best observation for each tag ID, independently in
the camera frame and `base_link`, with an individual timestamp on every pose.
The best observation is selected by lowest pose error, then highest decision
margin, lowest hamming distance, and newest timestamp.

- `apriltags/detections_camera` (`cbr_interfaces/AprilTagDetectionArray`) in
  `camera_optical_frame`;
- `apriltags/detections` (`cbr_interfaces/AprilTagDetectionArray`) in
  `base_link`.

Each detection keeps `family`, `id`, `decision_margin`, `hamming`,
`pose_error`, and `pose` together. The compatibility interfaces are:

- `apriltags/poses_camera` (`geometry_msgs/PoseArray`) in
  `camera_optical_frame`, published even before a TF to the robot is available;
- `apriltags/poses` (`geometry_msgs/PoseArray`) in `base_link`;
- TF frames parented directly to `camera_optical_frame` and named
  `apriltag_<family>_<id>`, such as `apriltag_tag36h11_5`. These remain
  independent of whether a time-aligned transform to `base_link` is available.

The detector uses the rectified projection matrix `CameraInfo.p` for `fx`,
`fy`, `cx`, and `cy`. Do not use a raw/distorted image; use `image_rect`.
