# CBR Vision

`vision` owns each on-demand camera session and exposes only
`/vision/analyze_scene`. A goal selects AprilTags, containers, or both with the
`requested_detectors` bit mask from `interfaces/action/AnalyzeScene`.

Both algorithms consume the same rectified frame, calibration and timestamp.
The node is the sole owner of camera capture and vision-light lifecycle.

```bash
ros2 launch vision vision.launch.py
```

AprilTags only:

```bash
ros2 action send_goal /vision/analyze_scene interfaces/action/AnalyzeScene \
  "{requested_detectors: 1, duration: {sec: 2, nanosec: 0}}" --feedback
```

Containers only use `requested_detectors: 2`; both use `3`.
White-table mapping uses `requested_detectors: 4`. Its goal contains only a
base-frame XY region, the work-surface height and a grid resolution. The result
is a row-major `TableSurfaceGrid` whose cells are free, blocked or unknown.
Each metric cell is projected onto the camera image and classified from every
pixel inside its quadrilateral, so perspective changes the pixel count without
changing the cell size in metres.
Vision has no knowledge of the gripper; table placement applies its footprint,
padding and yaw options to the returned grid.
For partial containers at the image edge, pass the work-surface height in the
base frame as `work_surface_height_m` (for example `0.125` for a 12.5 cm WS).
The table and container placement actions supply this value automatically.

Debug topics remain algorithm-specific:

- `/apriltags/debug_image`
- `/containers/debug_image`
- `/table_surface/debug_image` (green: usable white pixels; orange: unknown
  because the image is too dark or saturated)

After container perception finishes, manipulation publishes the exact TCP
release pose on `/manipulation/container_release_target`. The vision node
projects it over the stored observation frame and republishes
`/containers/debug_image` with a magenta diamond labelled `MoveIt TCP target`.
The projection deliberately uses the camera transform stored with that frame,
because the camera moves with the arm after planning starts.

The Bin 3 detector currently fits the 173 x 102 mm external silhouette. It
does not yet distinguish the 140 x 90 mm opening. Detection results carry the
configured external height; manipulation combines it with work-surface height
and its release offset. The physical drop through the opening still needs
verification on the robot.

Contours clipped by the image boundary are fitted against a projected
rectangle of the configured external size on the known top plane. A partial
fit carries `partial`, `position_uncertainty_m`, `yaw_uncertainty_deg` and
`partial_fit_overlap` in the detection; its debug-image label includes the
visible-silhouette overlap. Low-overlap fits remain table obstacles, while
`place_in_container` checks configurable target-quality limits.
The fit requires camera calibration, a base-frame transform and a visible
colored contour. The full-view PnP path is used for complete contours.
Contours within `container_border_margin_px` of an image edge remain partial
despite small mask fluctuations. During one analysis session, an exact TF is
preferred; if its timestamp is temporarily unavailable, the detector uses the
latest TF or the last transform obtained in that same session. The debug label
`partial_waiting_tf` identifies frames seen before any transform was available.

Container results are temporal tracks, not isolated contours. The detector
waits for camera/LED stabilization, requires repeated observations, rejects
position and yaw outliers, merges same-color tracks that converge, and returns
the median position. Base-frame yaw is normalized modulo 180 degrees because
the fitted external rectangle has that symmetry. The result fields
`observation_count`, `position_spread_m`, and `yaw_spread_deg` expose the
support and stability of every confirmed container.
