import json
import tempfile
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import patch
from route_editor import FORMAT, PreviewClock, RouteEditor, load_storyboard, save_storyboard, validate_steps


class ModelTests(unittest.TestCase):
    def test_preserves_order_and_duplicate_names(self):
        steps = [{"type": "waypoint", "label": "a1"}, {"type": "wait", "seconds": 15}, {"type": "waypoint", "label": "a1"}]
        self.assertEqual(validate_steps(steps), steps)

    def test_rejects_invalid_waits(self):
        for seconds in [True, "15", 0, -1, float("nan"), float("inf"), 86401]:
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                validate_steps([{"type": "wait", "seconds": seconds}])

    def test_rejects_unknown_steps(self):
        with self.assertRaises(ValueError):
            validate_steps([{"type": "unknown"}])

    def test_json_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "route.json"
            steps = [{"type": "waypoint", "label": "入口"}, {"type": "wait", "seconds": 0.25}]
            save_storyboard(target, steps)
            self.assertEqual(load_storyboard(target), steps)
            self.assertFalse(target.with_name("route.json.tmp").exists())

    def test_rejects_robot_route_format(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "route.json"
            target.write_text(json.dumps({"format": "pfa_online_slam_route/v1", "waypoints": []}))
            with self.assertRaises(ValueError):
                load_storyboard(target)

    def test_pause_excludes_paused_time(self):
        clock = PreviewClock()
        clock.start(15, 100)
        clock.pause(104)
        self.assertEqual(clock.left(200), 11)
        clock.resume(200)
        self.assertEqual(clock.left(205), 6)
        self.assertEqual(clock.left(220), 0)


class GuiTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(f"Display unavailable: {exc}")
        self.root.withdraw()
        self.editor = RouteEditor(self.root)

    def tearDown(self):
        if hasattr(self, "editor"):
            self.editor.cancel()
            self.root.destroy()

    def test_example_has_peer_wait_block(self):
        self.editor.example()
        self.assertEqual(len(self.editor.steps), 8)
        self.assertEqual(self.editor.steps[4], {"type": "wait", "seconds": 15})
        self.assertEqual(len(self.editor.table.get_children()), 8)

    def test_reorder_delete_wait(self):
        self.editor.example()
        self.editor.table.selection_set("4")
        self.editor.move(-1)
        self.assertEqual(self.editor.steps[3]["type"], "wait")
        self.editor.delete()
        self.assertEqual(len(self.editor.steps), 7)

    def test_preview_advances_pauses_and_cancels(self):
        self.editor.steps = [{"type": "waypoint", "label": "a1"}, {"type": "wait", "seconds": 15}]
        with patch("route_editor.time.monotonic", return_value=100):
            self.editor.play()
        with patch("route_editor.time.monotonic", return_value=101):
            self.root.after_cancel(self.editor.after_id)
            self.editor.after_id = None
            self.editor.tick()
        self.assertEqual(self.editor.index, 1)
        with patch("route_editor.time.monotonic", return_value=104):
            self.editor.toggle_pause()
        self.assertEqual(self.editor.clock.left(1000), 12)
        self.editor.cancel()
        self.assertFalse(self.editor.running)
        self.assertIsNone(self.editor.after_id)
        self.assertEqual(self.editor.index, 1)

    def test_preview_finishes_and_can_restart(self):
        self.editor.steps = [{"type": "wait", "seconds": 0.1}]
        with patch("route_editor.time.monotonic", return_value=100):
            self.editor.play()
        with patch("route_editor.time.monotonic", return_value=101):
            self.root.after_cancel(self.editor.after_id)
            self.editor.after_id = None
            self.editor.tick()
        self.assertFalse(self.editor.running)
        self.assertIn("完成", self.editor.status.get())
        self.editor.play()
        self.assertTrue(self.editor.running)


if __name__ == "__main__":
    unittest.main(verbosity=2)