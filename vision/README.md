# CBR Vision

`vision` owns each on-demand camera session and exposes only
`/vision/analyze_scene`. A goal selects AprilTags, containers, or both with the
`requested_detectors` bit mask from `interfaces/action/AnalyzeScene`.

The algorithms consume the same rectified camera stream and calibration. Each
detector has an independent worker and retains only its own newest pending
frame, so a slow container or table analysis neither builds an old-frame queue
nor blocks AprilTag observations. The node is the sole owner of camera capture
and vision-light lifecycle.

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
The planar region is rectified once per observation and all metric cells are
classified in one vectorized operation. Sampling density follows the projected
cell size (4 to 16 samples per axis), so perspective changes image coverage
without changing the cell size in metres.
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

During an active session, the AprilTag and container images show the effective
completed-frame rate, processed-frame count and base-transform coverage.  The
last image is retained with transient-local QoS and is replaced at the end by a
`FINAL` (or `CANCELED`) summary.  That summary draws only the base-frame
detections returned by the action, including an explicit zero-accepted result;
per-frame rejected candidates remain visible only in the live debug stream.
Container summaries include temporal support and spread, while AprilTag
summaries include pose error, decision margin and Hamming distance.

Processing budgets are independent: `apriltag_detection_rate_hz`,
`container_detection_rate_hz`, and `table_surface_detection_rate_hz` set the
maximum rate of each path. They are ceilings, not guaranteed throughput. Live
debug rendering has its own `debug_image_rate_hz`; reducing it does not change
detections or the final summary. `nthreads` controls pupil_apriltags and
`opencv_threads` controls OpenCV native workers.

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
preferred for every container; if its timestamp is temporarily unavailable,
the detector uses the latest TF or the last transform obtained in that same
session. This is valid because the arm remains stationary during scene
analysis. The debug label `partial_waiting_tf` identifies partial contours seen
before any transform was available.

Container results are temporal tracks, not isolated contours. The detector
waits for camera/LED stabilization, requires repeated observations, rejects
position and yaw outliers, merges same-color tracks that converge, and returns
the median position. Base-frame yaw is normalized modulo 180 degrees because
the fitted external rectangle has that symmetry. The result fields
`observation_count`, `position_spread_m`, and `yaw_spread_deg` expose the
support and stability of every confirmed container.

### Detector de contêiner por centro HSV

`AnalyzeScene.Goal.CONTAINERS_HSV` (bit 8) seleciona a alternativa por cor e
centro de componente na imagem retificada. O detector `CONTAINERS` (bit 2)
continua disponível. Os dois bits não podem ser pedidos na mesma análise,
pois compartilham os campos `best_containers_*` da action. O depósito usa
`CONTAINERS_HSV` por padrão.

A máscara usa os limites HSV existentes. O detector exige a área mínima em
pixels da faixa de altura da mesa (`<=7,5 cm`, `<=12,5 cm`, `>12,5 cm`)
para componentes completos. Componentes que tocam a borda usam faixas
independentes (`<=5 cm`, `<=10 cm`, `>10 cm`). Centros da mesma cor devem
estar a até `hsv_container_center_tolerance_px` em pelo menos
`hsv_container_min_confirmed_frames` imagens. O resultado em `base_link` usa
somente o pixel central e a interseção do raio da câmera com o plano
`work_surface_height_m + external_height_m` (0,073 m por padrão),
convertido do `floor_frame` para `base_frame` por TF. No robô móvel,
`floor_frame` é `base_footprint` e `base_frame` é `arm_base_link`.

Componentes cortados podem ser confirmados e retornados como `partial`.
No depósito com `CONTAINERS_HSV`, o alvo XY é o centro da parte visível,
sem estimativa do centro do contêiner inteiro. Os campos
`partial_fit_overlap=0` e `position_uncertainty_m=1` são marcadores de que
não houve ajuste geométrico; os limites de qualidade do detector antigo não
são aplicados ao HSV. O centro pode ficar deslocado da abertura. Ajuste as
áreas e o limite de proximidade em `config/vision.yaml` com gravações da
câmera, iluminação e alturas reais. A câmera e o braço devem permanecer
parados durante a janela de confirmação.
