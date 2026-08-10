# CBR AprilTag

This package subscribes to a **rectified** `sensor_msgs/Image` and its matching
`sensor_msgs/CameraInfo`, detects AprilTags, and uses `tf2` to publish their
poses in `base_link`.

The camera driver must publish an optical frame (for example,
`laptop_camera_optical_frame`) in the image header. The URDF provides this
frame from the physical `laptop_camera` link.

## Run

Install the Python detector in the ROS environment:

```bash
python3 -m pip install pupil-apriltags
```

Build and source the workspace, then launch:

```bash
colcon build --packages-select cbr_apriltag so_arm_101_description
source install/setup.bash
ros2 launch cbr_apriltag apriltag.launch.py \
  image_topic:=/camera/image_rect \
  camera_info_topic:=/camera/camera_info
```

The tag side is 32 mm by default. The published interfaces are:

- `apriltags/poses` (`geometry_msgs/PoseArray`) in `base_link`;
- TF frames named `apriltag_<family>_<id>`, such as `apriltag_tag36h11_5`.

The detector uses `CameraInfo.k` for `fx`, `fy`, `cx`, and `cy`. Do not use a
raw/distorted image with this calibration; use the camera driver's rectified
image topic or calibrate and rectify it first.
