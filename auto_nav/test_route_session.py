import json
import tempfile
import unittest
from pathlib import Path

from route_session import (
    cleanup_stale_sessions,
    create_session,
    finish_session,
    is_session_route,
    load_session,
    validate_session,
    validate_session_route,
)


class RouteSessionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name) / "sessions"

    def tearDown(self):
        self.temp.cleanup()

    def make_route(self, session, name="route.json"):
        path = session.routes_dir / name
        path.write_text('{"frame": "odom"}\n', encoding="utf-8")
        return path

    def test_create_session_is_unique_and_active(self):
        first = create_session(root=self.root)
        second = create_session(root=self.root)

        self.assertNotEqual(first.session_id, second.session_id)
        self.assertTrue(first.path.is_dir())
        self.assertTrue(first.routes_dir.is_dir())
        self.assertTrue(first.active)
        self.assertEqual(load_session(first.session_id, self.root).session_id, first.session_id)
        self.assertEqual(validate_session(first.session_id, self.root).path, first.path)

    def test_session_route_must_match_session(self):
        first = create_session(root=self.root)
        second = create_session(root=self.root)
        route = self.make_route(first)

        self.assertTrue(is_session_route(route, self.root))
        self.assertEqual(
            validate_session_route(route, first.session_id, self.root).session_id,
            first.session_id,
        )
        with self.assertRaisesRegex(ValueError, "不能由 session"):
            validate_session_route(route, second.session_id, self.root)

    def test_persistent_path_is_not_a_session_route(self):
        persistent = self.root.parent / "online_slam_goals" / "reality" / "routes" / "1.json"
        persistent.parent.mkdir(parents=True)
        persistent.write_text("{}", encoding="utf-8")

        self.assertFalse(is_session_route(persistent, self.root))
        with self.assertRaisesRegex(ValueError, "不是 session"):
            validate_session_route(persistent, "abc", self.root)

    def test_success_finish_removes_routes_and_session(self):
        session = create_session(root=self.root)
        self.make_route(session)

        finish_session(session, success=True)

        self.assertFalse(session.path.exists())

    def test_failure_keeps_diagnostics_but_removes_executable_route(self):
        session = create_session(root=self.root)
        route = self.make_route(session)

        finish_session(
            session,
            success=False,
            diagnostics={"reason": "tf missing", "session_id": session.session_id},
        )

        self.assertFalse(route.exists())
        self.assertTrue((session.diagnostics_dir / "failure.json").is_file())
        manifest = json.loads(
            (session.path / "manifest.json").read_text(encoding="utf-8")
        )
        self.assertEqual(manifest["status"], "failed")

    def test_failure_keeps_existing_diagnostics(self):
        session = create_session(root=self.root)
        self.make_route(session)
        existing = session.diagnostics_dir / "failure.json"
        existing.write_text('{"reason": "original"}\n', encoding="utf-8")

        finish_session(session, success=False, diagnostics=None)

        self.assertEqual(
            json.loads(existing.read_text(encoding="utf-8"))["reason"],
            "original",
        )

    def test_cleanup_stale_session_removes_routes_only(self):
        session = create_session(root=self.root)
        route = self.make_route(session)
        manifest_path = session.path / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["pid"] = 99999999
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

        cleaned = cleanup_stale_sessions(root=self.root)

        self.assertEqual(cleaned, [session.session_id])
        self.assertFalse(route.exists())
        self.assertTrue((session.path / "manifest.json").is_file())
        self.assertEqual(
            json.loads(manifest_path.read_text(encoding="utf-8"))["status"], "stale"
        )

    def test_cleanup_keeps_active_session(self):
        session = create_session(root=self.root)
        self.make_route(session)

        cleaned = cleanup_stale_sessions(root=self.root)

        self.assertEqual(cleaned, [])
        self.assertTrue((session.routes_dir / "route.json").is_file())

    def test_session_id_cannot_escape_root(self):
        with self.assertRaises(ValueError):
            load_session("../../etc", self.root)


if __name__ == "__main__":
    unittest.main()
