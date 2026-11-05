"""Shared route/map identity helpers.

Keep this module ROS-independent: the Tk GUI, tests, and ROS nodes all import
it.  A route that claims the ``map`` frame must carry the identity of the map
it was recorded against; coordinates alone are not enough.
"""

import hashlib
import math

ROUTE_FORMAT = "pfa_online_slam_route/v2"
ROUTE_FORMAT_V1 = "pfa_online_slam_route/v1"
SUPPORTED_FRAMES = ("map", "odom")
MAP_ID_VERSION = "pfa-map-v1"


def _finite_float(value, field):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} 必须是数字") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} 必须是有限数字")
    return result


def _positive_int(value, field):
    if isinstance(value, bool):
        raise ValueError(f"{field} 必须是正整数")
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} 必须是正整数") from exc
    if result <= 0:
        raise ValueError(f"{field} 必须是正整数")
    return result


def normalize_map_metadata(raw):
    """Validate and normalize a map identity document.

    Flat ``origin_x``/``origin_y``/``origin_z``/``origin_yaw`` keys and the
    nested ``origin`` form are both accepted so JSON point files and YAML map
    headers can use the same validator.
    """
    if not isinstance(raw, dict):
        raise ValueError("地图身份必须是对象")
    map_id = raw.get("map_id")
    if not isinstance(map_id, str) or not map_id.strip():
        raise ValueError("地图身份缺少 map_id")
    metadata = {"map_id": map_id.strip()}

    if "width" in raw:
        metadata["width"] = _positive_int(raw["width"], "地图 width")
    if "height" in raw:
        metadata["height"] = _positive_int(raw["height"], "地图 height")
    if "resolution" in raw:
        resolution = _finite_float(raw["resolution"], "地图 resolution")
        if resolution <= 0.0:
            raise ValueError("地图 resolution 必须大于 0")
        metadata["resolution"] = resolution

    origin = raw.get("origin")
    if origin is not None and not isinstance(origin, dict):
        raise ValueError("地图 origin 必须是对象")
    for key in ("x", "y", "z", "yaw"):
        flat_key = f"origin_{key}"
        if flat_key in raw:
            metadata[flat_key] = _finite_float(raw[flat_key], flat_key)
        elif isinstance(origin, dict) and key in origin:
            metadata[flat_key] = _finite_float(origin[key], f"origin.{key}")
    return metadata


def _origin_yaw(quaternion):
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def map_metadata_from_occupancy(info, data):
    """Return the stable identity/metadata of an ``OccupancyGrid`` message."""
    width = _positive_int(info.width, "地图 width")
    height = _positive_int(info.height, "地图 height")
    resolution = _finite_float(info.resolution, "地图 resolution")
    if resolution <= 0.0:
        raise ValueError("地图 resolution 必须大于 0")

    origin = info.origin
    origin_x = float(origin.position.x)
    origin_y = float(origin.position.y)
    origin_z = float(origin.position.z)
    origin_yaw = _origin_yaw(origin.orientation)

    try:
        payload = bytes(data)
    except (TypeError, ValueError):
        payload = bytes(int(value) & 0xFF for value in data)

    digest = hashlib.sha256()
    digest.update(MAP_ID_VERSION.encode("ascii"))
    digest.update(
        (
            f"|{width}|{height}|{resolution:.17g}"
            f"|{origin_x:.17g}|{origin_y:.17g}|{origin_z:.17g}"
            f"|{origin_yaw:.17g}|{len(payload)}"
        ).encode("ascii")
    )
    digest.update(payload)

    return {
        "map_id": digest.hexdigest()[:16],
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "origin_z": origin_z,
        "origin_yaw": origin_yaw,
    }


def map_id_matches(current, expected):
    """Return True only when both identities exist and are equal."""
    if not current or not expected:
        return False
    return current.get("map_id") == expected.get("map_id")


def yaml_map_header(frame, map_metadata=None):
    """Build the route/map header lines written above Waypoints_Num."""
    if frame not in SUPPORTED_FRAMES:
        raise ValueError("路线 frame 必须是 map 或 odom")
    lines = [
        f"Route_Format: {ROUTE_FORMAT}",
        f"Route_Frame: {frame}",
    ]
    if map_metadata is not None:
        metadata = normalize_map_metadata(map_metadata)
        lines.append("Map:")
        lines.append(f"  map_id: {metadata['map_id']}")
        for key in (
            "width",
            "height",
            "resolution",
            "origin_x",
            "origin_y",
            "origin_z",
            "origin_yaw",
        ):
            if key in metadata:
                value = metadata[key]
                if isinstance(value, float):
                    value = f"{value:.12g}"
                lines.append(f"  {key}: {value}")
    return lines


def parse_yaml_map_header(data):
    """Return ``(frame, metadata, legacy)`` from a loaded YAML document.

    Files without ``Route_Frame`` are legacy waypoint files.  They are treated
    as map-frame data by the caller, but the caller must warn because they
    carry no map identity.
    """
    if not isinstance(data, dict):
        raise ValueError("航点 YAML 必须是对象")
    raw_frame = data.get("Route_Frame")
    if raw_frame is None:
        return None, None, True
    raw_format = data.get("Route_Format")
    if raw_format is not None and raw_format != ROUTE_FORMAT:
        raise ValueError(f"航点 YAML 的 Route_Format 必须是 {ROUTE_FORMAT}")
    if raw_frame not in SUPPORTED_FRAMES:
        raise ValueError("航点 YAML 的 Route_Frame 必须是 map 或 odom")
    raw_map = data.get("Map")
    metadata = normalize_map_metadata(raw_map) if raw_map is not None else None
    return raw_frame, metadata, False
