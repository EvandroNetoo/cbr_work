# CBR Container Detection

This package detects the external colored silhouette of known red and blue
Bin 3 containers. Detection is on demand and uses the same rectified camera,
camera calibration, arm-mounted TF and vision light used by the AprilTag
detector.

The first implementation deliberately does **not** distinguish the inner
opening from the external wall. It fits the measured 173 x 102 mm external
rectangle to each accepted color contour. The 73 mm external height and all
internal dimensions are recorded in the profile for the later manipulation
integration, but are not used to calculate the pose yet.

## Run

Start the normal camera/robot processing, then launch the detector:

```bash
colcon build --symlink-install --packages-select interfaces container_detection
source install/setup.bash
ros2 launch container_detection container_detection.launch.py
```

Move the arm to the existing `detect_apriltags` named state before requesting
an analysis. A two-second test is:

```bash
ros2 action send_goal /containers/analyze \
  interfaces/action/AnalyzeContainers \
  "{duration: {sec: 2, nanosec: 0}}" --feedback
```

Inspect `/containers/debug_image` with `rqt_image_view`. The overlay contains:

- translucent red and blue threshold masks;
- the raw contour in the detected color;
- the fitted external quadrilateral and its four corners;
- green for an accepted pose and orange for a rejected candidate;
- area (`A`), rectangularity (`R`) and reprojection error (`E`);
- a permanent `exterior-only MVP` notice.

Detections are published in the camera and robot frames as
`/containers/detections_camera` and `/containers/detections`. Initial HSV
thresholds in `config/container_detection.yaml` must be calibrated with images
from the real camera and lighting before manipulation is enabled.

## Current boundary

This package does not move the arm, select a placement pose, add MoveIt
collision geometry or enable `PlaceInContainer`. A colored side wall can bias
the exterior-only pose. The debug experiment exists to measure that bias before
the inner opening and safety margins are implemented.
