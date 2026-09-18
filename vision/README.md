# Unified vision

`vision` is the only application node that leases the wrist camera capture and
the vision LED. It serves `interfaces/action/SceneAnalyzer` at
`/vision/analyze`; AprilTags and containers requested by one goal are evaluated
from the same image callback and `CameraInfo`.

The output pose frame defaults to `arm_base_link`, always retaining the image
timestamp. TF is looked up at that timestamp with a finite timeout; there is no
fallback to the latest transform. A container pose is the centre of its opening
on the upper rim plane. Its X axis follows physical depth, Y follows width, Z
points away from the table, and yaw is axial (equivalent modulo 180 degrees).
The goal's nonnegative `table_height_m` is measured above
`work_surface_height_frame` (`base_footprint`, at floor level), not along the
output-frame Z axis. At the image timestamp the node transforms that floor
plane to `arm_base_link`; for 5/10 cm tables its Z can legitimately be
negative, because the composed robot's arm origin is 112 mm above the floor.
Missing or tilted floor TF rejects container candidates (tag analysis may
still complete).

## Geometry profiles

The active `current_team_model` describes the team's physical model:

- external depth × width × height: 173 × 102 × 73 mm;
- internal depth × width × height: 140 × 90 × 57 mm.

It must not be confused with the distinct `robocup_2026_type_28` object from
the rulebook: 160 × 135 × 82 mm external and 125 × 120 × 65 mm internal. Both
are in `config/container_geometry_profiles.yaml`, but only the team model is
active by default. Changing profiles is an explicit physical-object change,
not a calibration adjustment.

Rulebook 2026 sections 3.6.1/3.6.5 specify table heights of 5, 10 and 15 cm
and the type-28 dimensions above. Section 3.8 specifies the colored 42 mm
AprilTag cubes; 5.4.2/5.4.3 and 5.5 require both colors and up to two cubes in
each bin. Section 6.5 penalizes contact with environmental objects. These are
competition requirements, not measurements of the team's model or gripper.

## Detection and calibration

The initial OpenCV HSV thresholds use its native hue scale `[0,179]`: S≥80,
V≥45, red 0–12 or 150–179 and blue 70–138. Morphology, visible-area ratio,
rim/opening support, temporal observations, segmentability of rim/sides and
partial-visibility limits are configurable in `config/vision.yaml`.

Projected area is computed from the 3-D open model, calibrated camera matrix,
camera pose, requested table height and image clipping. The expected colored
area includes visible outer faces and the rim but excludes the dark opening;
therefore no monotonic area assumption is made for 5, 10 or 15 cm tables.
The opening score tolerates the maximum projected area of zero, one or two
42 mm cubes, while the outer geometry and rim remain mandatory.
The aperture contour, rather than the colored-pixel centroid, determines the
deposition XY/yaw. If the boundary disappears behind occlusion, the node
rejects the candidate rather than extrapolating hidden edges from a color
fragment. Adjacent fused containers may likewise be rejected until separately
observed; this is intentional fail-closed behavior.

Debug images are disabled by default. Enable `publish_debug_images` to publish
`vision/debug/apriltags` and `vision/debug/containers`.

The following values are engineering defaults, not measurements on the final
robot: segmentability factors, confidence thresholds, uncertainty model,
morphology sizes and hardware idle grace. They require captures using the real
camera, LED, both physical container colors, empty openings, one/two cubes,
shadows, reflections and image-edge occlusions.

On this workspace use `python3 -m venv --system-site-packages .venv` and
`.venv/bin/python -m pip install pupil-apriltags`. OpenCV and NumPy are provided
by `python3-opencv` and `python3-numpy`. The launch discovers the workspace
`.venv`; `python_executable` or `CBR_VISION_PYTHON` can override it.

## Action example

```bash
ros2 action send_goal /vision/analyze interfaces/action/SceneAnalyzer \
  "{analyze_apriltags: true, analyze_containers: true, table_height_m: 0.10, duration: {sec: 2}}"
```

A zero duration is continuous until cancellation. Goals with neither mode,
negative/non-finite height, negative duration, or while another session owns
the hardware are rejected. Camera and LED release is guaranteed after success,
abort, cancellation, exception and shutdown; a short configurable grace period
allows consecutive goals to reuse the lease.
