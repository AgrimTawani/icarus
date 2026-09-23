"""Normalize AirSim sensor evidence without exposing simulator truth."""

import math
import time

from python.simulation.airsim_rpc import MsgpackRpcClient

SCHEMA = "icarus.simulation.airsim-sensors.v1"


def normalize_snapshot(state, images, lidar, now_unix_ms=None,
                       clearance_m=2.5):
    """Convert AirSim NED sensor results to compact Icarus evidence.

    Raw images, point clouds, collision truth, segmentation labels and poses
    are intentionally not returned. Pixel processing belongs upstream of this
    boundary; DCM curation receives only compact perception facts.
    """
    now_unix_ms = now_unix_ms or int(time.time() * 1000)
    timestamp_ns = min(
        [value for value in [state.get("timestamp"), lidar.get("time_stamp")]
         + [image.get("time_stamp") for image in images]
         if type(value) is int and value > 0], default=0)
    # Classic environments differ: some expose Unix nanoseconds and others a
    # simulation-monotonic counter. A synchronous successful RPC is fresh at
    # receipt; never mislabel a monotonic value as a 1970 Unix timestamp.
    timestamp_domain = ("unix_ns" if timestamp_ns >= 1_000_000_000_000_000
                        else "simulator_monotonic_ns")
    observed_ms = (timestamp_ns // 1_000_000
                   if timestamp_domain == "unix_ns" else now_unix_ms)
    points = lidar.get("point_cloud") or []
    triples = [points[i:i + 3] for i in range(0, len(points) - 2, 3)]
    finite = [point for point in triples
              if len(point) == 3 and all(math.isfinite(v) for v in point)]
    nearest = min(finite, key=lambda p: math.hypot(p[0], p[1]), default=None)
    nearest_distance = (math.hypot(nearest[0], nearest[1])
                        if nearest is not None else None)
    bearing = (math.degrees(math.atan2(nearest[1], nearest[0])) % 360
               if nearest is not None else None)
    image_metadata = [{"type": image.get("image_type"),
                       "width": image.get("width"),
                       "height": image.get("height"),
                       "timestamp_ns": image.get("time_stamp")}
                      for image in images]
    return {
        "schema": SCHEMA,
        "frame": "NED",
        "observed_at_unix_ms": observed_ms,
        "received_at_unix_ms": now_unix_ms,
        "state": {"observed_at_unix_ms": observed_ms,
                  "landed_state": state.get("landed_state")},
        "perception": {
            "observed_at_unix_ms": observed_ms,
            "overall": ("PERCEPTION_HEALTH_OK" if observed_ms
                        else "PERCEPTION_HEALTH_UNAVAILABLE"),
            "local_map_available": bool(finite),
            "local_map_age_ms": max(0, now_unix_ms - observed_ms) if observed_ms else None,
            "path_ahead_clear": (nearest_distance >= clearance_m
                                 if nearest_distance is not None else None),
            "nearest_obstacle_distance_m": (round(nearest_distance, 3)
                                            if nearest_distance is not None else None),
            "nearest_obstacle_bearing_deg": (round(bearing, 3)
                                             if bearing is not None else None),
        },
        "sensor_metadata": {
            "images": image_metadata,
            "lidar_sensor": "Lidar1",
            "lidar_points": len(finite),
            "units": {"distance": "m", "angle": "deg", "timestamp": "ns"},
            "timestamp_domain": timestamp_domain,
        },
        "truth_fields_excluded": ["collision", "pose", "segmentation", "object_labels"],
    }


class AirSimSensorAdapter:
    def __init__(self, host="127.0.0.1", port=41451, vehicle="Icarus",
                 lidar="Lidar1", timeout=10):
        self.rpc = MsgpackRpcClient(host, port, timeout)
        self.vehicle = vehicle
        self.lidar_name = lidar

    def close(self):
        self.rpc.close()

    def snapshot(self):
        requests = [
            {"camera_name": "0", "image_type": 0,
             "pixels_as_float": False, "compress": True},
            {"camera_name": "0", "image_type": 1,
             "pixels_as_float": True, "compress": False},
        ]
        return normalize_snapshot(
            self.rpc.multirotor_state(self.vehicle),
            self.rpc.images(requests, self.vehicle),
            self.rpc.lidar(self.lidar_name, self.vehicle))
