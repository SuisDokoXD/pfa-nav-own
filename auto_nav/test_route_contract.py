import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml

from online_slam_goal_gui import build_route_payload, build_waypoints_yaml
from route_contract import (
    map_id_matches,
    map_metadata_from_occupancy,
    normalize_map_metadata,
    parse_yaml_map_header,
    yaml_map_header,
)


def occupancy_info(width=4, height=3, resolution=0.05, origin=(1.0, 2.0, 0.0)):
    x, y, z = origin
    return SimpleNamespace(
        width=width,
        height=height,
        resolution=resolution,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


MAP_METADATA = {
    "map_id": "abc123",
    "width": 4,
    "height": 3,
    "resolution": 0.05,
    "origin_x": 1.0,
    "origin_y": 2.0,
    "origin_z": 0.0,
    "origin_yaw": 0.0,
}


class MapIdentityTest(unittest.TestCase):
    def test_map_id_is_deterministic_and_content_sensitive(self):
        info = occupancy_info()
        first = map_metadata_from_occupancy(info, [-1, 0, 100, 0, 0, 0])
        second = map_metadata_from_occupancy(info, [-1, 0, 100, 0, 0, 0])
        changed = map_metadata_from_occupancy(info, [-1, 0, 0, 0, 0, 0])

        self.assertEqual(first["map_id"], second["map_id"])
        self.assertNotEqual(first["map_id"], changed["map_id"])
        self.assertEqual(first["width"], 4)
        self.assertEqual(first["resolution"], 0.05)

    def test_normalize_rejects_missing_map_id(self):
        with self.assertRaisesRegex(ValueError, "map_id"):
            normalize_map_metadata({"width": 4})

    def test_map_id_match_requires_both_sides(self):
        self.assertTrue(
            map_id_matches({"map_id": "a"}, {"map_id": "a"})
        )
        self.assertFalse(map_id_matches({"map_id": "a"}, {"map_id": "b"}))
        self.assertFalse(map_id_matches(None, {"map_id": "a"}))

    def test_yaml_header_round_trip(self):
        lines = yaml_map_header("map", MAP_METADATA)
        data = yaml.safe_load("\n".join(lines))
        frame, metadata, legacy = parse_yaml_map_header(data)

        self.assertEqual(frame, "map")
        self.assertFalse(legacy)
        self.assertEqual(metadata["map_id"], "abc123")

    def test_legacy_yaml_is_reported_as_legacy(self):
        frame, metadata, legacy = parse_yaml_map_header(
            {"Waypoints_Num": 1, "Waypoint_1": {}}
        )
        self.assertIsNone(frame)
        self.assertIsNone(metadata)
        self.assertTrue(legacy)


class GuiRouteContractTest(unittest.TestCase):
    def write_goal(self, directory, name, frame="map", map_metadata=None):
        path = Path(directory) / f"{name}.json"
        payload = {"frame": frame, "x": 1.0, "y": 2.0, "yaw": 0.0}
        if map_metadata is not None:
            payload["map"] = map_metadata
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_map_route_requires_map_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            point = self.write_goal(directory, "a", frame="map")
            with self.assertRaisesRegex(ValueError, "map_id"):
                build_route_payload(
                    "route", [{"type": "waypoint", "path": point}]
                )

    def test_map_route_carries_identity_and_exports_yaml(self):
        with tempfile.TemporaryDirectory() as directory:
            first = self.write_goal(directory, "a", map_metadata=MAP_METADATA)
            second = self.write_goal(directory, "b", map_metadata=MAP_METADATA)
            payload = build_route_payload(
                "route",
                [
                    {"type": "waypoint", "path": first},
                    {"type": "waypoint", "path": second},
                ],
            )
            text = build_waypoints_yaml(
                payload["steps"],
                frame=payload["frame"],
                map_metadata=payload["map"],
                source="route.json",
            )

        data = yaml.safe_load(text)
        self.assertEqual(payload["map"]["map_id"], "abc123")
        self.assertEqual(data["Route_Frame"], "map")
        self.assertEqual(data["Map"]["map_id"], "abc123")
        self.assertEqual(data["Waypoints_Num"], 2)
        self.assertAlmostEqual(data["Waypoint_1"]["Pos_x"], 1.0)

    def test_mixed_map_identities_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            first = self.write_goal(directory, "a", map_metadata=MAP_METADATA)
            other = dict(MAP_METADATA, map_id="other")
            second = self.write_goal(directory, "b", map_metadata=other)
            with self.assertRaisesRegex(ValueError, "不同地图"):
                build_route_payload(
                    "route",
                    [
                        {"type": "waypoint", "path": first},
                        {"type": "waypoint", "path": second},
                    ],
                )

    def test_odom_route_cannot_export_auto_nav_yaml(self):
        with tempfile.TemporaryDirectory() as directory:
            point = self.write_goal(directory, "a", frame="odom")
            payload = build_route_payload(
                "route", [{"type": "waypoint", "path": point}]
            )

        self.assertEqual(payload["frame"], "odom")
        self.assertNotIn("map", payload)
        with self.assertRaisesRegex(ValueError, "只有 map"):
            build_waypoints_yaml(payload["steps"], frame="odom")


if __name__ == "__main__":
    unittest.main()
