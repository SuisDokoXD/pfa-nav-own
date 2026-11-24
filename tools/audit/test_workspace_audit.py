"""workspace_audit.py 的 unittest 测试（标准库，无第三方依赖）。

运行: PYTHONDONTWRITEBYTECODE=1 TMPDIR=$PI_SCRATCH_DIR python3 -B -m unittest -v test_workspace_audit

安全前提：所有临时目录由 tempfile.mkdtemp() 落在 TMPDIR 下；被审计样例中的
.sh / .py 只作为数据写入，测试断言它们从未被执行。
"""

import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "workspace_audit.py")
sys.path.insert(0, HERE)
import workspace_audit  # noqa: E402  (被测对象本身，不是被审计工程)


def write_file(path, text):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


def package_xml(name, deps=(), version="0.1.0"):
    lines = [
        '<?xml version="1.0"?>',
        '<package format="3">',
        "  <name>%s</name>" % name,
        "  <version>%s</version>" % version,
    ]
    lines.extend("  <depend>%s</depend>" % dep for dep in deps)
    lines.append("</package>")
    return "\n".join(lines) + "\n"


def run_tool(*args, env=None):
    environ = dict(os.environ)
    environ["PYTHONDONTWRITEBYTECODE"] = "1"
    if env:
        environ.update(env)
    return subprocess.run(
        [sys.executable, "-B", TOOL, *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=180,
        env=environ,
    )


def parse_report(proc):
    return json.loads(proc.stdout)


class WorkspaceCase(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="ws_audit_test_")
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)
        tmpdir = os.path.realpath(tempfile.gettempdir())
        self.assertTrue(
            os.path.realpath(self.root).startswith(tmpdir),
            "临时工作区必须位于 TMPDIR 下: %s" % self.root,
        )

    def path(self, *parts):
        return os.path.join(self.root, *parts)

    def put(self, rel, text):
        write_file(self.path(rel), text)

    def put_layout_root(self):
        os.makedirs(self.path("src", "pb2025_sentry_nav", "pb2025_nav_bringup"), exist_ok=True)

    def put_all_assets(self):
        self.put("install/setup.bash", "# built\n")
        self.put("src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/game.yaml", "image: game.pgm\n")
        self.put("src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/game.pgm", "P5\n1 1\n255\n\x00")
        self.put("src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/game.pcd", "VERSION 0.7\n")

    def sum_warnings(self, report):
        return report["summary"]["warnings"]

    def codes(self, report):
        return [item["code"] for item in report["findings"]]


class CleanWorkspaceTests(WorkspaceCase):
    def test_packages_dependencies_and_assets_pass(self):
        self.put_layout_root()
        self.put_all_assets()
        self.put("src/nav_bringup/package.xml", package_xml("nav_bringup", ["nav2_msgs", "local_utils"]))
        self.put("src/nav_utils/package.xml", package_xml("local_utils", ["rclcpp"]))
        self.put("src/nav_bringup/launch/bringup.launch.py", "VALUE = 1\n")
        self.put("src/nav_bringup/scripts/run.sh", "set -e\necho ok\n")

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = parse_report(proc)

        self.assertEqual(report["summary"]["errors"], 0)
        self.assertEqual(report["summary"]["warnings"], 0)
        self.assertEqual(report["summary"]["packages_found"], 2)
        self.assertEqual([p["name"] for p in report["packages"]], ["local_utils", "nav_bringup"])
        by_name = {p["name"]: p for p in report["packages"]}
        self.assertEqual(by_name["nav_bringup"]["local_dependencies"], ["local_utils"])
        self.assertEqual(by_name["local_utils"]["local_dependencies"], [])
        self.assertIn(
            {"name": "nav2_msgs", "type": "depend"},
            by_name["nav_bringup"]["declared_dependencies"],
        )
        self.assertEqual(by_name["nav_bringup"]["path"], "src/nav_bringup")

        statuses = {check["id"]: check["status"] for check in report["checks"]}
        self.assertEqual(statuses["workspace_layout"], "passed")
        self.assertEqual(statuses["ros_packages"], "passed")
        self.assertEqual(statuses["python_syntax"], "passed")
        self.assertEqual(statuses["shell_syntax"], "passed")
        self.assertEqual(statuses["required_assets"], "passed")
        self.assertEqual(report["summary"]["checks"]["passed"], 5)
        self.assertEqual(report["summary"]["checks"]["failed"], 0)
        self.assertEqual(report["summary"]["checks"]["skipped"], 0)


class ErrorTests(WorkspaceCase):
    def test_bad_package_xml_is_error(self):
        self.put_layout_root()
        self.put("src/broken/package.xml", "<package><name>broken</name>\n")

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 1)
        report = parse_report(proc)
        self.assertIn("PACKAGE_XML_INVALID", self.codes(report))
        statuses = {check["id"]: check["status"] for check in report["checks"]}
        self.assertEqual(statuses["ros_packages"], "failed")

    def test_missing_package_name_is_error(self):
        self.put_layout_root()
        self.put("src/nameless/package.xml", '<?xml version="1.0"?><package format="3"><version>1</version></package>')

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 1)
        self.assertIn("PACKAGE_NAME_MISSING", self.codes(parse_report(proc)))

    def test_duplicate_package_name_is_error(self):
        self.put_layout_root()
        self.put("src/a/package.xml", package_xml("duplicated"))
        self.put("src/b/package.xml", package_xml("duplicated"))

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 1)
        report = parse_report(proc)
        duplicates = [item for item in report["findings"] if item["code"] == "PACKAGE_NAME_DUPLICATE"]
        self.assertEqual(len(duplicates), 1)
        self.assertIn("src/a", duplicates[0]["message"])
        self.assertIn("src/b", duplicates[0]["message"])

    def test_python_syntax_error_is_error(self):
        self.put_layout_root()
        self.put("src/x/scripts/broken.py", "def broken(:\n    return 1\n")

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 1)
        report = parse_report(proc)
        finding = [item for item in report["findings"] if item["code"] == "PYTHON_SYNTAX"]
        self.assertEqual(len(finding), 1)
        self.assertEqual(finding[0]["path"], "src/x/scripts/broken.py")
        self.assertIn("第 1 行", finding[0]["message"])

    def test_shell_syntax_error_is_error(self):
        self.put_layout_root()
        self.put("src/x/scripts/broken.sh", "if true; then\n  echo hi\n")

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 1)
        report = parse_report(proc)
        finding = [item for item in report["findings"] if item["code"] == "SHELL_SYNTAX"]
        self.assertEqual(len(finding), 1)
        self.assertEqual(finding[0]["path"], "src/x/scripts/broken.sh")


class AssetTests(WorkspaceCase):
    def test_missing_assets_warn_without_failing(self):
        self.put_layout_root()
        self.put("src/x/package.xml", package_xml("x"))

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0)
        report = parse_report(proc)
        self.assertEqual(report["summary"]["errors"], 0)
        self.assertEqual(self.sum_warnings(report), 4)
        self.assertEqual(set(self.codes(report)), {"ASSET_MISSING"})
        paths = {item["path"] for item in report["findings"]}
        self.assertIn("install/setup.bash", paths)
        self.assertIn("src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/game.yaml", paths)
        self.assertIn("src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/game.pgm", paths)
        self.assertIn("src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/game.pcd", paths)
        statuses = {check["id"]: check["status"] for check in report["checks"]}
        self.assertEqual(statuses["required_assets"], "warning")
        self.assertNotEqual(statuses["required_assets"], "passed")

    def test_empty_asset_is_warning_not_error(self):
        self.put_layout_root()
        self.put_all_assets()
        open(self.path("src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/game.pcd"), "w").close()

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0)
        report = parse_report(proc)
        self.assertIn("ASSET_EMPTY", self.codes(report))
        self.assertEqual(report["summary"]["errors"], 0)

    def test_unknown_layout_is_not_applicable(self):
        self.put("README.md", "not a ros workspace\n")
        self.put("tools/local.py", "print('hi')\n")

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0)
        report = parse_report(proc)
        self.assertEqual(report["findings"], [])
        statuses = {check["id"]: check["status"] for check in report["checks"]}
        self.assertEqual(statuses["required_assets"], "not_applicable")
        self.assertEqual(statuses["ros_packages"], "not_applicable")
        self.assertEqual(statuses["python_syntax"], "passed")
        self.assertGreaterEqual(report["summary"]["checks"]["not_applicable"], 2)
        self.assertEqual(report["summary"]["checks"]["passed"] + report["summary"]["checks"]["not_applicable"], 5)


class ScopeTests(WorkspaceCase):
    def test_excluded_dirs_and_symlinks_are_skipped(self):
        self.put_layout_root()
        self.put("src/x/package.xml", package_xml("x"))
        self.put("src/x/ok.py", "VALUE = 1\n")
        for excluded in (".git", "build", "install", "log", "__pycache__"):
            self.put("%s/broken.py" % excluded, "def broken(:\n")
            self.put("%s/broken.sh" % excluded, "if true; then\n")

        outside = tempfile.mkdtemp(prefix="ws_audit_outside_")
        self.addCleanup(shutil.rmtree, outside, ignore_errors=True)
        write_file(os.path.join(outside, "broken.py"), "def broken(:\n")
        os.symlink(outside, self.path("src", "linked_dir"))
        os.symlink(os.path.join(outside, "broken.py"), self.path("src", "x", "linked.py"))

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = parse_report(proc)
        self.assertEqual(set(self.codes(report)), {"ASSET_MISSING"})
        self.assertGreaterEqual(report["summary"]["symlinks_skipped"], 2)
        scanned = [p["name"] for p in report["packages"]]
        self.assertEqual(scanned, ["x"])

    def test_project_scripts_are_never_executed(self):
        self.put_layout_root()
        self.put("src/x/package.xml", package_xml("x"))
        marker = self.path("marker_shell.txt")
        self.put(
            "src/x/scripts/danger.sh",
            '#!/usr/bin/env bash\ntouch "%s"\necho never-run\n' % marker,
        )
        py_marker = self.path("marker_python.txt")
        self.put(
            "src/x/scripts/danger.py",
            "open(%r, 'w', encoding='utf-8').write('never-run')\n" % py_marker,
        )

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertFalse(os.path.exists(marker), "bash -n 之外不得执行工程脚本")
        self.assertFalse(os.path.exists(py_marker), ".py 不得被导入或执行")

    def test_missing_bash_marks_shell_skipped(self):
        self.put_layout_root()
        self.put("src/x/package.xml", package_xml("x"))
        self.put("src/x/scripts/ok.sh", "echo ok\n")
        empty_bin = tempfile.mkdtemp(prefix="ws_audit_nobash_")
        self.addCleanup(shutil.rmtree, empty_bin, ignore_errors=True)

        proc = run_tool("--root", self.root, env={"PATH": empty_bin})
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = parse_report(proc)
        statuses = {check["id"]: check["status"] for check in report["checks"]}
        self.assertEqual(statuses["shell_syntax"], "skipped")
        self.assertNotIn("SHELL_SYNTAX", self.codes(report))
        self.assertEqual(report["summary"]["checks"]["skipped"], 1)
        self.assertEqual(report["summary"]["errors"], 0)


class OutputTests(WorkspaceCase):
    def test_default_prints_stdout_and_writes_nothing(self):
        self.put("README.md", "plain\n")
        before = sorted(
            os.path.relpath(os.path.join(dirpath, name), self.root)
            for dirpath, _, names in os.walk(self.root)
            for name in names
        )

        proc = run_tool("--root", self.root)
        after = sorted(
            os.path.relpath(os.path.join(dirpath, name), self.root)
            for dirpath, _, names in os.walk(self.root)
            for name in names
        )
        self.assertEqual(before, after, "默认模式不得写入被审计目录")
        report = parse_report(proc)
        self.assertEqual(report["root"], os.path.realpath(self.root))
        self.assertIn("notes", report)

    def test_output_file_is_written_and_stdout_silent(self):
        self.put("README.md", "plain\n")
        out = self.path("audit.json")

        proc = run_tool("--root", self.root, "--output", out)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(proc.stdout, "")
        with open(out, encoding="utf-8") as handle:
            report = json.load(handle)
        self.assertEqual(report["tool"], "workspace_audit")
        self.assertEqual(report["schema_version"], workspace_audit.SCHEMA_VERSION)

    def test_output_into_missing_directory_exits_2(self):
        self.put("README.md", "plain\n")
        out = self.path("missing_dir", "audit.json")

        proc = run_tool("--root", self.root, "--output", out)
        self.assertEqual(proc.returncode, 2)
        self.assertIn("失败", proc.stderr)
        self.assertFalse(os.path.exists(self.path("missing_dir")))

    def test_existing_output_survives_failed_replace(self):
        out = self.path("audit.json")
        write_file(out, "ORIGINAL\n")

        with mock.patch.object(workspace_audit.os, "replace", side_effect=OSError("boom")):
            with self.assertRaises(OSError):
                workspace_audit.write_json_atomic(out, '{"new": true}\n')

        with open(out, encoding="utf-8") as handle:
            self.assertEqual(handle.read(), "ORIGINAL\n")
        leftovers = [n for n in os.listdir(self.root) if n.startswith(".workspace_audit-")]
        self.assertEqual(leftovers, [], "临时文件必须清理")

    def test_write_replaces_existing_file(self):
        out = self.path("audit.json")
        write_file(out, "ORIGINAL\n")
        workspace_audit.write_json_atomic(out, "REPLACED\n")
        with open(out, encoding="utf-8") as handle:
            self.assertEqual(handle.read(), "REPLACED\n")
        self.assertEqual([n for n in os.listdir(self.root) if n.startswith(".workspace_audit-")], [])


class CliFailureTests(WorkspaceCase):
    def test_missing_root_exits_2(self):
        proc = run_tool("--root", self.path("does_not_exist"))
        self.assertEqual(proc.returncode, 2)
        self.assertIn("不是已存在的目录", proc.stderr)
        self.assertEqual(proc.stdout, "")

    def test_root_that_is_a_file_exits_2(self):
        target = self.path("plain.txt")
        write_file(target, "x\n")
        proc = run_tool("--root", target)
        self.assertEqual(proc.returncode, 2)

    def test_missing_required_argument_exits_2(self):
        proc = run_tool()
        self.assertEqual(proc.returncode, 2)


class DeterminismTests(WorkspaceCase):
    def test_report_is_byte_identical_across_runs(self):
        self.put_layout_root()
        self.put("src/b/package.xml", package_xml("b", ["a"]))
        self.put("src/a/package.xml", package_xml("a"))
        self.put("src/a/scripts/z.sh", "echo z\n")
        self.put("src/b/scripts/y.py", "VALUE = 2\n")

        first = run_tool("--root", self.root)
        second = run_tool("--root", self.root)
        self.assertEqual(first.stdout, second.stdout)
        report = parse_report(first)
        keys = [(item["code"], item["path"], item["message"]) for item in report["findings"]]
        self.assertEqual(keys, sorted(keys))


class UnreadableFileTests(WorkspaceCase):
    def test_unreadable_python_file_is_reported_not_passed(self):
        if hasattr(os, "geteuid") and os.geteuid() == 0:
            self.skipTest("root 用户无法构造不可读文件")
        self.put_layout_root()
        self.put("src/x/package.xml", package_xml("x"))
        target = self.path("src/x/scripts/locked.py")
        self.put("src/x/scripts/locked.py", "VALUE = 1\n")
        os.chmod(target, 0o000)
        self.addCleanup(os.chmod, target, 0o644)

        report = workspace_audit.audit(os.path.realpath(self.root))
        self.assertIn("FILE_UNREADABLE", [item["code"] for item in report["findings"]])
        statuses = {check["id"]: check["status"] for check in report["checks"]}
        self.assertEqual(statuses["python_syntax"], "skipped")



class AssetPathSymlinkTests(WorkspaceCase):
    """资产/布局路径逐级符号链接检测：链接一律不可验证，绝不 passed。"""

    def statuses(self, report):
        return {check["id"]: check["status"] for check in report["checks"]}

    def assert_assets_unverified(self, report):
        self.assertEqual(report["summary"]["errors"], 0)
        codes = [item["code"] for item in report["findings"]]
        self.assertIn("ASSET_UNVERIFIED", codes)
        status = self.statuses(report)["required_assets"]
        self.assertEqual(status, "skipped")
        self.assertNotIn(status, ("passed", "not_applicable"))

    def test_asset_symlink_pointing_inside_root_is_unverified(self):
        self.put_layout_root()
        self.put_all_assets()
        os.remove(self.path("src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/game.pcd"))
        self.put("src/elsewhere/real.pcd", "VERSION 0.7\n")
        os.symlink(
            self.path("src/elsewhere/real.pcd"),
            self.path("src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/game.pcd"),
        )

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = parse_report(proc)
        self.assert_assets_unverified(report)
        finding = [item for item in report["findings"] if item["code"] == "ASSET_UNVERIFIED"]
        self.assertEqual(finding[0]["path"], "src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/game.pcd")
        self.assertIn("符号链接", finding[0]["message"])

    def test_asset_symlink_to_outside_root_is_unverified(self):
        self.put_layout_root()
        self.put_all_assets()
        outside = tempfile.mkdtemp(prefix="ws_audit_asset_outside_")
        self.addCleanup(shutil.rmtree, outside, ignore_errors=True)
        write_file(os.path.join(outside, "outside.yaml"), "image: outside.pgm\n")
        os.remove(self.path("src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/game.yaml"))
        os.symlink(
            os.path.join(outside, "outside.yaml"),
            self.path("src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/game.yaml"),
        )

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assert_assets_unverified(parse_report(proc))

    def test_asset_parent_dir_symlink_cannot_escape_root(self):
        self.put_layout_root()
        self.put_all_assets()
        outside = tempfile.mkdtemp(prefix="ws_audit_dir_outside_")
        self.addCleanup(shutil.rmtree, outside, ignore_errors=True)
        write_file(os.path.join(outside, "game.pcd"), "VERSION 0.7\n")
        pcd_dir = self.path("src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality")
        shutil.rmtree(pcd_dir)
        os.symlink(outside, pcd_dir)

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = parse_report(proc)
        self.assert_assets_unverified(report)
        finding = [item for item in report["findings"] if item["code"] == "ASSET_UNVERIFIED"]
        self.assertIn("reality", finding[0]["message"])

    def test_layout_root_symlink_is_unverified_not_applicable(self):
        outside = tempfile.mkdtemp(prefix="ws_audit_layout_outside_")
        self.addCleanup(shutil.rmtree, outside, ignore_errors=True)
        write_file(
            os.path.join(outside, "pb2025_nav_bringup", "map", "reality", "game.yaml"), "image: game.pgm\n"
        )
        os.makedirs(self.path("src"))
        os.symlink(outside, self.path("src", "pb2025_sentry_nav"))

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = parse_report(proc)
        codes = [item["code"] for item in report["findings"]]
        self.assertIn("LAYOUT_UNVERIFIED", codes)
        statuses = self.statuses(report)
        self.assertEqual(statuses["workspace_layout"], "skipped")
        self.assertEqual(statuses["required_assets"], "skipped")
        self.assertNotIn(statuses["required_assets"], ("passed", "not_applicable"))


class TraversalFailureTests(WorkspaceCase):
    """os.walk 的 onerror 必须被报告：扫描不完整不允许静默通过。"""

    def fake_walk(self, root, onerror=None, followlinks=False):
        if onerror is not None:
            onerror(PermissionError(13, "Permission denied", os.path.join(root, "src", "blocked")))
        yield root, [], []

    def test_onerror_is_reported_and_scan_marked_incomplete(self):
        self.put("README.md", "plain\n")
        with mock.patch.object(workspace_audit.os, "walk", self.fake_walk):
            report = workspace_audit.audit(self.root)
        self.assertEqual(report["summary"]["traversal_failures"], 1)
        self.assertFalse(report["summary"]["scan_complete"])
        self.assertGreaterEqual(report["summary"]["warnings"], 1, "不允许 errors=0 且无警告")
        finding = [item for item in report["findings"] if item["code"] == "TRAVERSAL_FAILED"]
        self.assertEqual(finding[0]["path"], "src/blocked")
        statuses = {check["id"]: check["status"] for check in report["checks"]}
        self.assertEqual(statuses["workspace_layout"], "skipped")

    def test_onerror_makes_cli_exit_2_and_still_emits_report(self):
        self.put("README.md", "plain\n")
        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(workspace_audit.os, "walk", self.fake_walk):
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                code = workspace_audit.main(["--root", self.root])
        self.assertEqual(code, 2)
        self.assertIn("扫描不完整", err.getvalue())
        self.assertFalse(json.loads(out.getvalue())["summary"]["scan_complete"])

    def test_real_permission_denied_directory(self):
        if hasattr(os, "geteuid") and os.geteuid() == 0:
            self.skipTest("root 用户无法构造不可读目录")
        blocked = self.path("src", "blocked")
        self.put("src/blocked/inside.py", "VALUE = 1\n")
        os.chmod(blocked, 0o000)
        self.addCleanup(os.chmod, blocked, 0o755)

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 2, proc.stderr)
        report = parse_report(proc)
        self.assertEqual(report["summary"]["traversal_failures"], 1)
        self.assertFalse(report["summary"]["scan_complete"])
        self.assertEqual(report["summary"]["errors"], 0)
        self.assertGreaterEqual(report["summary"]["warnings"], 1)
        self.assertIn("TRAVERSAL_FAILED", [item["code"] for item in report["findings"]])


class PackageXmlStructureTests(WorkspaceCase):
    def test_root_tag_must_be_package(self):
        self.put_layout_root()
        self.put(
            "src/notpkg/package.xml",
            '<?xml version="1.0"?>\n<not_package>\n  <name>notpkg</name>\n  <depend>rclcpp</depend>\n</not_package>\n',
        )

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 1)
        report = parse_report(proc)
        self.assertIn("PACKAGE_ROOT_TAG_INVALID", [item["code"] for item in report["findings"]])
        self.assertEqual(report["summary"]["packages_found"], 0)

    def test_condition_is_preserved_but_not_evaluated(self):
        self.put_layout_root()
        self.put("src/conditional/package.xml", package_xml("conditional", ["cond_dep"]))
        self.put("src/cond_dep/package.xml", package_xml("cond_dep"))
        self.put(
            "src/conditional/package.xml",
            '<?xml version="1.0"?>\n<package format="3">\n  <name>conditional</name>\n'
            "  <buildtool_export_depend>eigen</buildtool_export_depend>\n"
            '  <depend condition="$ROS_VERSION == 2">cond_dep</depend>\n'
            "  <depend>rclpy</depend>\n</package>\n",
        )

        report = workspace_audit.audit(os.path.realpath(self.root))
        entry = {pkg["name"]: pkg for pkg in report["packages"]}["conditional"]
        deps = {dep["name"]: dep for dep in entry["declared_dependencies"]}
        self.assertEqual(deps["eigen"]["type"], "buildtool_export_depend")
        self.assertEqual(deps["cond_dep"]["condition"], "$ROS_VERSION == 2")
        self.assertNotIn("condition", deps["rclpy"])
        # 声明图：条件依赖同样计入本地依赖，未求值 condition
        self.assertEqual(entry["local_dependencies"], ["cond_dep"])


class ShellCheckerTests(WorkspaceCase):
    def tally_and_files(self):
        self.put("src/x/scripts/a.sh", "echo ok\n")
        path = self.path("src/x/scripts/a.sh")
        return workspace_audit.CheckTally("shell_syntax"), [("src/x/scripts/a.sh", path)]

    def codes(self, tally):
        return [item["code"] for item in tally.findings]

    def test_timeout_is_skipped_not_error(self):
        tally, files = self.tally_and_files()
        with mock.patch.object(
            workspace_audit.subprocess, "run", side_effect=subprocess.TimeoutExpired("bash", 10)
        ):
            workspace_audit.scan_shell(files, tally)
        self.assertEqual(tally.status(), "skipped")
        self.assertEqual(self.codes(tally), [])

    def test_launcher_failure_is_skipped_not_syntax(self):
        tally, files = self.tally_and_files()
        with mock.patch.object(workspace_audit.subprocess, "run", side_effect=OSError(13, "Permission denied")):
            workspace_audit.scan_shell(files, tally)
        self.assertEqual(tally.status(), "skipped")
        self.assertEqual(self.codes(tally), ["SHELL_CHECKER_UNAVAILABLE"])
        self.assertNotIn("SHELL_SYNTAX", self.codes(tally))

    def test_non_syntax_returncodes_are_not_reported_as_syntax(self):
        for returncode in (1, 126, 127):
            tally, files = self.tally_and_files()
            completed = mock.Mock(returncode=returncode, stderr=b"bash: x: cannot read\n")
            with mock.patch.object(workspace_audit.subprocess, "run", return_value=completed):
                workspace_audit.scan_shell(files, tally)
            self.assertNotIn("SHELL_SYNTAX", self.codes(tally), "rc=%d" % returncode)
            self.assertEqual(tally.status(), "skipped", "rc=%d" % returncode)

    def test_checker_returns_2_is_syntax_error(self):
        tally, files = self.tally_and_files()
        completed = mock.Mock(returncode=2, stderr=b"a.sh: line 2: syntax error\n")
        with mock.patch.object(workspace_audit.subprocess, "run", return_value=completed):
            workspace_audit.scan_shell(files, tally)
        self.assertEqual(self.codes(tally), ["SHELL_SYNTAX"])
        self.assertEqual(tally.status(), "failed")

    def test_checker_env_excludes_shell_startup_variables(self):
        tally, files = self.tally_and_files()
        completed = mock.Mock(returncode=0, stderr=b"")
        with mock.patch.object(workspace_audit.subprocess, "run", return_value=completed) as run:
            workspace_audit.scan_shell(files, tally)
        env = run.call_args.kwargs["env"]
        for hostile in ("BASH_ENV", "ENV", "BASHOPTS", "SHELLOPTS", "CDPATH", "PS4", "PROMPT_COMMAND"):
            self.assertNotIn(hostile, env)
        self.assertIn("PATH", env)
        self.assertEqual(env["LC_ALL"], "C")
    def test_hostile_parent_env_does_not_change_result(self):
        self.put_layout_root()
        self.put("src/x/scripts/ok.sh", "echo ok\n")
        outside = tempfile.mkdtemp(prefix="ws_audit_env_")
        self.addCleanup(shutil.rmtree, outside, ignore_errors=True)
        bad_env = os.path.join(outside, "bad_env.sh")
        write_file(bad_env, "if true; then\n")

        proc = run_tool("--root", self.root, env={"BASH_ENV": bad_env, "SHELLOPTS": "posix", "ENV": bad_env})
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = parse_report(proc)
        self.assertNotIn("SHELL_SYNTAX", [item["code"] for item in report["findings"]])
        statuses = {check["id"]: check["status"] for check in report["checks"]}
        self.assertEqual(statuses["shell_syntax"], "passed")

    def test_unreadable_script_is_skipped_not_syntax(self):
        if hasattr(os, "geteuid") and os.geteuid() == 0:
            self.skipTest("root 用户无法构造不可读文件")
        self.put_layout_root()
        target = self.path("src/x/scripts/locked.sh")
        self.put("src/x/scripts/locked.sh", "echo hi\n")
        os.chmod(target, 0o000)
        self.addCleanup(os.chmod, target, 0o644)

        report = workspace_audit.audit(os.path.realpath(self.root))
        codes = [item["code"] for item in report["findings"]]
        self.assertIn("FILE_UNREADABLE", codes)
        self.assertNotIn("SHELL_SYNTAX", codes, "不可读不是语法错误")
        statuses = {check["id"]: check["status"] for check in report["checks"]}
        self.assertEqual(statuses["shell_syntax"], "skipped")


class PackageApplicabilityTests(WorkspaceCase):
    """适用性只能由"是否发现 manifest"决定，不能由"成功解析数"决定。"""

    def status(self, report, check_id):
        return {check["id"]: check["status"] for check in report["checks"]}[check_id]

    def detail(self, report, check_id):
        return {check["id"]: check["detail"] for check in report["checks"]}[check_id]

    def test_all_manifests_unreadable_is_skipped_not_not_applicable(self):
        if hasattr(os, "geteuid") and os.geteuid() == 0:
            self.skipTest("root 用户无法构造不可读文件")
        # 不建布局根：避免 assets 的 ASSET_MISSING 警告混入本用例计数
        for name in ("a", "b"):
            rel = "src/%s/package.xml" % name
            self.put(rel, package_xml(name))
            target = self.path(rel)
            os.chmod(target, 0o000)
            self.addCleanup(os.chmod, target, 0o644)

        report = workspace_audit.audit(os.path.realpath(self.root))
        codes = [item["code"] for item in report["findings"]]
        self.assertEqual(codes.count("FILE_UNREADABLE"), 2)
        self.assertEqual(report["summary"]["warnings"], 2)
        self.assertEqual(report["summary"]["packages_found"], 0)
        status = self.status(report, "ros_packages")
        self.assertEqual(status, "skipped")
        self.assertNotEqual(status, "not_applicable")
        self.assertIn("发现 2 个 package.xml，成功解析 0 个", self.detail(report, "ros_packages"))

    def test_partially_unreadable_is_skipped_not_passed(self):
        if hasattr(os, "geteuid") and os.geteuid() == 0:
            self.skipTest("root 用户无法构造不可读文件")
        # 不建布局根（同上）
        self.put("src/ok/package.xml", package_xml("ok"))
        self.put("src/locked/package.xml", package_xml("locked"))
        target = self.path("src/locked/package.xml")
        os.chmod(target, 0o000)
        self.addCleanup(os.chmod, target, 0o644)

        report = workspace_audit.audit(os.path.realpath(self.root))
        self.assertEqual(report["summary"]["packages_found"], 1)
        self.assertEqual([item["code"] for item in report["findings"]], ["FILE_UNREADABLE"])
        status = self.status(report, "ros_packages")
        self.assertEqual(status, "skipped")
        self.assertNotEqual(status, "passed")

    def test_no_manifest_at_all_is_not_applicable(self):
        self.put("README.md", "plain\n")
        self.put("src/x/main.py", "VALUE = 1\n")

        report = workspace_audit.audit(os.path.realpath(self.root))
        self.assertEqual(self.status(report, "ros_packages"), "not_applicable")
        self.assertEqual(report["summary"]["warnings"], 0)
        self.assertEqual([item for item in report["findings"] if "PACKAGE" in item["code"]], [])

    def test_bad_xml_is_still_failed(self):
        self.put_layout_root()
        self.put("src/broken/package.xml", "<package><name>broken</name>\n")

        proc = run_tool("--root", self.root)
        self.assertEqual(proc.returncode, 1)
        report = parse_report(proc)
        self.assertEqual(self.status(report, "ros_packages"), "failed")
        self.assertIn("PACKAGE_XML_INVALID", [item["code"] for item in report["findings"]])


class DocBoundaryTests(unittest.TestCase):
    def read(self, name):
        with open(os.path.join(HERE, name), encoding="utf-8") as handle:
            return handle.read()

    def test_readme_documents_static_tree_and_layered_acceptance(self):
        text = self.read("README.md")
        for phrase in ("静态工作树", "安全全停", "隔离硬件", "rosbag"):
            self.assertIn(phrase, text)
        self.assertNotIn("可上车性必须由带 rosbag 的定量实验", text)

    def test_tasks_marks_asset_state_unknown_and_forbids_auto_edits(self):
        text = self.read("TASKS.md")
        for phrase in ("状态可能为未知", "不得据此修改", "不自动修改 YAML", "未来需授权"):
            self.assertIn(phrase, text)
if __name__ == "__main__":
    unittest.main()
