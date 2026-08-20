from cbr_lidar.lidar_driver import LidarScan
from cbr_lidar.lidar_node import _scan_start_age_ns


def test_scan_timestamp_keeps_last_ray_at_acquisition_end():
    scan = LidarScan(
        sequence=1,
        ranges_m=(1.0,) * 121,
        rpm=274.0,
        start_monotonic_ns=1_900_000_000,
        end_monotonic_ns=1_990_000_000,
    )
    now_ns = 2_000_000_000

    start_age_ns = _scan_start_age_ns(scan, len(scan.ranges_m), now_ns)
    ray_step_ns = (60.0 / scan.rpm) / 360.0 * 1e9
    last_ray_age_ns = start_age_ns - round(ray_step_ns * 120)

    assert last_ray_age_ns == now_ns - scan.end_monotonic_ns
