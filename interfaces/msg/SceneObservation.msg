uint8 APRILTAGS=1
uint8 CONTAINERS=2
uint8 TABLE_SURFACE=4

# Detectors requested for this semantic operation. A completed observation may
# legitimately contain no detections.
uint8 requested_detectors
bool completed
interfaces/AprilTagStampedDetection[] apriltags
interfaces/ContainerStampedDetection[] containers
