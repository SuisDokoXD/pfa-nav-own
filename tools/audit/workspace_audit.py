#!/usr/bin/env python3
"""离线只读工作区审计 CLI（仅标准库，Python 3.8+）。

用途：在不导入、不执行、不修改被审计工程的前提下，收集 ROS 工作区的结构证据：
package.xml 声明依赖与本地依赖、Python 语法、Shell 语法、已知导航入口资产的存留。

明确边界（结论不得超出这些范围）：
  * 不 import / exec 任何被审计代码；.py 只用 ast 解析。
  * .sh 只跑 `bash -n`（仅语法解析），绝不执行；bash 缺失或超时记 skipped，绝不算 passed。
  * 不解析 YAML，不加载 PGM/PCD，不证明地图与点云匹配。
  * 不启动 ROS，不联网，不安装依赖，不修复任何 C++/配置/launch。
  * 静态工作树假设：审计期间假定工作区不被并发修改；不做 TOCTOU 防护，本工具不是安全沙箱。
  * 退出码 0 只表示"未发现 error"，不代表可上车，也不代表 LIO/TF/GICP/NAV2/底盘正确。
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

SCHEMA_VERSION = 1
TOOL_NAME = "workspace_audit"
EXCLUDED_DIR_NAMES = frozenset({".git", "build", "install", "log", "__pycache__"})
DEPENDENCY_TAGS = frozenset(
    (
        "depend",
        "build_depend",
        "build_export_depend",
        "exec_depend",
        "buildtool_depend",
        "buildtool_export_depend",
        "test_depend",
        "doc_depend",
    )
)
EXPECTED_PACKAGE_ROOT_TAG = "package"
NAV_LAYOUT_ROOT = "src/pb2025_sentry_nav"
BRINGUP = NAV_LAYOUT_ROOT + "/pb2025_nav_bringup"
# 已知导航入口依赖；只验证存在与非空，不解析内容。
REQUIRED_ASSETS = (
    ("bringup_setup", "install/setup.bash", "导航入口 bringup 的构建产物"),
    ("map_yaml", BRINGUP + "/map/reality/game.yaml", "比赛地图元数据"),
    ("map_pgm", BRINGUP + "/map/reality/game.pgm", "game.yaml 约定默认栅格地图"),
    ("pcd", BRINGUP + "/pcd/reality/game.pcd", "定位先验点云"),
)
SHELL_TIMEOUT_SECONDS = 10
# bash -n 的返回码：0 通过；2 语法错误；126/127 是"文件不可用"而不是语法问题。
SHELL_SYNTAX_RETURNCODE = 2
BUG_PROBE_NOTE = "未做浅字符串算法探测；LIO/TF/GICP/NAV2/底盘行为未经本工具验证"


class CheckTally:
    """一个检查项的计数与逐条诊断。

    status 优先级: failed > not_applicable > skipped > warning > passed。
    即：出现 error 记 failed；无输入记 not_applicable；有未验证项记 skipped
    （跳过不能算通过）；仅有 warning 记 warning；否则 passed。
    """

    def __init__(self, check_id, applicable=True):
        self.id = check_id
        self.applicable = applicable
        self.errors = 0
        self.warnings = 0
        self.skipped = 0
        self.details = []
        self.findings = []

    def record(self, severity, code, path, message):
        self.findings.append({"severity": severity, "code": code, "path": path, "message": message})
        if severity == "error":
            self.errors += 1
        else:
            self.warnings += 1

    def status(self):
        if self.errors:
            return "failed"
        if not self.applicable:
            return "not_applicable"
        if self.skipped:
            return "skipped"
        if self.warnings:
            return "warning"
        return "passed"

    def as_dict(self):
        return {
            "id": self.id,
            "status": self.status(),
            "detail": "; ".join(self.details) if self.details else "无发现",
        }


def _local_tag(tag):
    return tag.rsplit("}", 1)[-1]


def _inside_root(root, path):
    try:
        return os.path.commonpath([root, os.path.realpath(path)]) == root
    except ValueError:
        return False


def check_relative_path(root, rel, expect="file"):
    """逐级 lstat 检查 root/rel 的每个组成部分；任何符号链接都视为不可验证。

    指向 root 内部的链接同样跳过（策略一致性），并且全过程 follow_symlinks=False，
    因此不存在"经由父目录符号链接逃出 root"的路径。expect ∈ file / dir。
    返回 (state, detail)，state ∈ ok / missing / empty / symlink / not_a_file / not_a_dir / unreadable。
    """
    current = root
    parts = [part for part in rel.split("/") if part]
    for index, part in enumerate(parts):
        current = os.path.join(current, part)
        try:
            info = os.stat(current, follow_symlinks=False)
        except FileNotFoundError:
            return "missing", "%s 不存在" % rel
        except OSError as exc:
            return "unreadable", "%s 无法 stat: %s" % (rel, exc.strerror or exc)
        if stat.S_ISLNK(info.st_mode):
            return "symlink", "%s 的组成部分 %r 是符号链接（按策略跳过）" % (rel, part)
        if not _inside_root(root, current):
            return "symlink", "%s 的真实路径越出 root" % rel
        is_last = index == len(parts) - 1
        if is_last and expect == "dir":
            if not stat.S_ISDIR(info.st_mode):
                return "not_a_dir", "%s 不是目录" % rel
        elif is_last:
            if not stat.S_ISREG(info.st_mode):
                return "not_a_file", "%s 不是常规文件" % rel
            if info.st_size == 0:
                return "empty", "%s 为空文件" % rel
        elif not stat.S_ISDIR(info.st_mode):
            return "not_a_dir", "%s 的父级 %r 不是目录" % (rel, part)
    return "ok", rel


def iter_workspace_files(root):
    """返回 (files, symlinks_skipped, failures)。

    不跟随符号链接（文件与目录都跳过），跳过排除目录，保证不越出 root，顺序稳定。
    目录读取失败（EACCES 等）通过 os.walk 的 onerror 收集，绝不静默忽略。
    """
    files = []
    symlinks_skipped = 0
    failures = []

    for dirpath, dirnames, filenames in os.walk(root, followlinks=False, onerror=failures.append):
        kept = []
        for name in sorted(dirnames):
            full = os.path.join(dirpath, name)
            if os.path.islink(full) or not _inside_root(root, full):
                symlinks_skipped += 1
            elif name not in EXCLUDED_DIR_NAMES:
                kept.append(name)
        dirnames[:] = kept
        for name in sorted(filenames):
            full = os.path.join(dirpath, name)
            if os.path.islink(full) or not _inside_root(root, full):
                symlinks_skipped += 1
            elif os.path.isfile(full):
                files.append((os.path.relpath(full, root).replace(os.sep, "/"), full))
    return files, symlinks_skipped, failures


def _rel_or_placeholder(root, path):
    if not path:
        return "<unknown>"
    try:
        rel = os.path.relpath(path, root).replace(os.sep, "/")
    except ValueError:
        return "<outside-root>"
    return rel if not rel.startswith("..") else "<outside-root>"


def scan_packages(files, tally):
    packages = []
    by_name = {}
    manifests = 0
    for rel, full in files:
        if os.path.basename(rel) != "package.xml":
            continue
        manifests += 1
        try:
            element = ET.parse(full).getroot()
        except ET.ParseError as exc:
            tally.record("error", "PACKAGE_XML_INVALID", rel, "XML 解析失败: %s" % exc)
            continue
        except OSError as exc:
            tally.skipped += 1
            tally.record("warning", "FILE_UNREADABLE", rel, "无法读取: %s" % (exc.strerror or exc))
            continue
        root_tag = _local_tag(element.tag) if isinstance(element.tag, str) else "<non-element>"
        if root_tag != EXPECTED_PACKAGE_ROOT_TAG:
            tally.record(
                "error",
                "PACKAGE_ROOT_TAG_INVALID",
                rel,
                "根元素应为 <%s>，实际为 <%s>" % (EXPECTED_PACKAGE_ROOT_TAG, root_tag),
            )
            continue
        name = version = None
        deps = []
        for child in element:
            tag = _local_tag(child.tag) if isinstance(child.tag, str) else ""
            text = (child.text or "").strip()
            if tag == "name" and name is None:
                name = text
            elif tag == "version" and version is None:
                version = text or None
            elif tag in DEPENDENCY_TAGS:
                if not text:
                    tally.record("warning", "PACKAGE_DEPENDENCY_EMPTY", rel, "<%s> 依赖缺少名称" % tag)
                    continue
                dep = {"name": text, "type": tag}
                condition = (child.get("condition") or "").strip()
                if condition:
                    # 只保留条件原文，不求值：声明图 != 运行图。
                    dep["condition"] = condition
                deps.append(dep)
        if not name:
            tally.record("error", "PACKAGE_NAME_MISSING", rel, "package.xml 缺少非空 <name>")
            continue
        entry = {
            "name": name,
            "version": version,
            "path": os.path.dirname(rel),
            "declared_dependencies": deps,
            "local_dependencies": [],
        }
        packages.append(entry)
        by_name.setdefault(name, []).append(entry)

    duplicates = 0
    for name in sorted(by_name):
        paths = sorted(entry["path"] for entry in by_name[name])
        if len(paths) > 1:
            duplicates += 1
            tally.record(
                "error", "PACKAGE_NAME_DUPLICATE", paths[0], "包名 %r 重复出现于: %s" % (name, ", ".join(paths))
            )

    known = set(by_name)
    for entry in packages:
        declared = {dep["name"] for dep in entry["declared_dependencies"]}
        entry["local_dependencies"] = sorted(declared & known - {entry["name"]})

    packages.sort(key=lambda item: (item["name"], item["path"]))
    # 适用性只看"发现了多少 manifest"：全部不可读时必须保持 applicable，
    # 否则 not_applicable 会遮蔽 skipped / FILE_UNREADABLE（未验证被误报成"不适用"）。
    if not manifests:
        tally.applicable = False
    else:
        tally.details.append(
            "发现 %d 个 package.xml，成功解析 %d 个，%d 个重名" % (manifests, len(packages), duplicates)
        )
    return packages


def scan_python(files, tally):
    targets = [(rel, full) for rel, full in files if rel.endswith(".py")]
    if not targets:
        tally.applicable = False
        return
    parsed = 0
    for rel, full in targets:
        try:
            with open(full, "rb") as handle:
                source = handle.read()
        except OSError as exc:
            tally.skipped += 1
            tally.record("warning", "FILE_UNREADABLE", rel, "无法读取: %s" % (exc.strerror or exc))
            continue
        try:
            ast.parse(source, filename=rel)
        except SyntaxError as exc:
            tally.record("error", "PYTHON_SYNTAX", rel, "第 %s 行: %s" % (exc.lineno, exc.msg))
        except (ValueError, RecursionError) as exc:
            tally.record("error", "PYTHON_PARSE_FAILED", rel, "ast 解析失败: %s" % exc)
        else:
            parsed += 1
    tally.details.append("%d/%d 个 .py 通过 ast 解析（不导入）" % (parsed, len(targets)))


def shell_checker_env(bash_path):
    """最小环境：不继承 BASH_ENV/ENV/BASHOPTS/SHELLOPTS 等 shell 启动配置。

    实测 `bash -n` 确实会读取 BASH_ENV，因此必须显式隔离，否则父进程环境会污染语法判定。
    """
    bindir = os.path.dirname(bash_path)
    path = os.pathsep.join([p for p in (bindir, os.defpath) if p])
    return {"PATH": path, "LC_ALL": "C", "LANG": "C", "TERM": "dumb"}


def scan_shell(files, tally):
    targets = [(rel, full) for rel, full in files if rel.endswith(".sh")]
    if not targets:
        tally.applicable = False
        return
    bash = shutil.which("bash")
    if bash is None:
        tally.skipped += len(targets)
        tally.details.append("未找到 bash，%d 个 .sh 未验证（不算通过）" % len(targets))
        return
    env = shell_checker_env(bash)
    ok = 0
    for rel, full in targets:
        try:
            with open(full, "rb") as handle:
                handle.read(1)
        except OSError as exc:
            # 脚本本身不可读：这是 IO 问题，不能报成语法错误。
            tally.skipped += 1
            tally.record("warning", "FILE_UNREADABLE", rel, "无法读取（未做语法检查）: %s" % (exc.strerror or exc))
            continue
        try:
            proc = subprocess.run(
                [bash, "-n", full],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=SHELL_TIMEOUT_SECONDS,
                env=env,
            )
        except subprocess.TimeoutExpired:
            tally.skipped += 1
            tally.details.append("%s: bash -n 超时（%ss）未验证" % (rel, SHELL_TIMEOUT_SECONDS))
            continue
        except OSError as exc:
            # bash 不可执行/无法启动：同样是未验证，不是语法错误。
            tally.skipped += 1
            tally.record(
                "warning",
                "SHELL_CHECKER_UNAVAILABLE",
                rel,
                "无法启动 bash（%s），未做语法检查" % (exc.strerror or exc),
            )
            continue
        if proc.returncode == 0:
            ok += 1
            continue
        lines = proc.stderr.decode("utf-8", "replace").strip().splitlines()
        first = lines[0] if lines else "(无 stderr)"
        if proc.returncode == SHELL_SYNTAX_RETURNCODE:
            tally.record("error", "SHELL_SYNTAX", rel, first)
        else:
            tally.skipped += 1
            tally.record(
                "warning",
                "SHELL_CHECK_FAILED",
                rel,
                "bash -n 退出码 %d（非语法错误，未验证）: %s" % (proc.returncode, first),
            )
    tally.details.append("%d/%d 个 .sh 通过 bash -n 语法解析（不执行）" % (ok, len(targets)))


def scan_assets(root, tally, layout_state):
    if layout_state == "missing":
        tally.applicable = False
        tally.details.append("未知工作区布局，资产检查不适用（不等于通过）")
        return
    if layout_state != "ok":
        # 符号链接/stat 失败：无法验证，不是"不适用"，更不是通过。
        tally.skipped += 1
        tally.details.append("布局根不可验证（%s），资产检查未完成，不计为通过" % layout_state)
        return
    present = 0
    for asset_id, rel, why in REQUIRED_ASSETS:
        state, detail = check_relative_path(root, rel)
        if state == "ok":
            present += 1
        elif state == "missing":
            tally.record("warning", "ASSET_MISSING", rel, "%s：文件不存在" % why)
        elif state == "empty":
            tally.record("warning", "ASSET_EMPTY", rel, "%s：文件为空（%s）" % (why, asset_id))
        else:
            tally.skipped += 1
            tally.record("warning", "ASSET_UNVERIFIED", rel, "%s：%s（未验证，不算通过）" % (why, detail))
    tally.details.append("%d/%d 个已知导航入口资产存在且非空（未解析内容）" % (present, len(REQUIRED_ASSETS)))


def audit(root):
    root = os.path.realpath(root)
    files, symlinks_skipped, failures = iter_workspace_files(root)
    layout_state, layout_detail = check_relative_path(root, NAV_LAYOUT_ROOT, expect="dir")
    layout = CheckTally("workspace_layout", applicable=layout_state != "missing")
    layout.details.append("布局根 %s: %s" % (NAV_LAYOUT_ROOT, layout_state))
    layout.details.append(
        "已扫描 %d 个文件；排除目录: %s；跳过符号链接 %d 个"
        % (len(files), ",".join(sorted(EXCLUDED_DIR_NAMES)), symlinks_skipped)
    )
    if layout_state == "missing":
        layout.details.append("未发现 %s，按未知布局处理（不适用，不等于通过）" % NAV_LAYOUT_ROOT)
    elif layout_state != "ok":
        layout.skipped += 1
        layout.record(
            "warning", "LAYOUT_UNVERIFIED", NAV_LAYOUT_ROOT, "布局根无法验证（%s）：%s" % (layout_state, layout_detail)
        )
    if failures:
        # 扫描不完整时不允许声称"不适用"或"通过"。
        layout.applicable = True
    for exc in failures:
        layout.skipped += 1
        layout.record(
            "warning",
            "TRAVERSAL_FAILED",
            _rel_or_placeholder(root, getattr(exc, "filename", None)),
            "目录读取失败，扫描不完整: %s" % (getattr(exc, "strerror", None) or exc),
        )
    if failures:
        layout.details.append("%d 处目录无法读取，本次扫描不完整" % len(failures))

    tallies = [layout]
    for check_id in ("ros_packages", "python_syntax", "shell_syntax", "required_assets"):
        tallies.append(CheckTally(check_id))
    packages_tally, python_tally, shell_tally, assets_tally = tallies[1:]

    packages = scan_packages(files, packages_tally)
    scan_python(files, python_tally)
    scan_shell(files, shell_tally)
    scan_assets(root, assets_tally, layout_state)

    checks = [tally.as_dict() for tally in tallies]
    counts = {"passed": 0, "failed": 0, "skipped": 0, "warning": 0, "not_applicable": 0}
    for check in checks:
        counts[check["status"]] += 1
    findings = sorted(
        (item for tally in tallies for item in tally.findings),
        key=lambda item: (item["code"], item["path"], item["message"]),
    )

    return {
        "schema_version": SCHEMA_VERSION,
        "tool": TOOL_NAME,
        "root": root,
        "summary": {
            "errors": sum(1 for item in findings if item["severity"] == "error"),
            "warnings": sum(1 for item in findings if item["severity"] == "warning"),
            "checks": counts,
            "files_scanned": len(files),
            "symlinks_skipped": symlinks_skipped,
            "packages_found": len(packages),
            "traversal_failures": len(failures),
            "scan_complete": not failures,
        },
        "checks": checks,
        "packages": packages,
        "findings": findings,
        "notes": [
            "只读审计：除 --output 指定路径外不写任何文件，不导入也不执行被审计代码。",
            ".sh 仅用 bash -n 做语法解析（隔离 BASH_ENV/ENV/BASHOPTS/SHELLOPTS）；跳过/超时不计通过。",
            "资产检查只验证存在与非空，未解析 YAML、未加载 PGM/PCD、未证明地图与点云匹配。",
            "local_dependencies 是 package.xml 的声明图，未求值 condition，不代表运行期真实依赖图。",
            "静态工作树假设：审计期间假定工作区不被并发修改；不做 TOCTOU 防护，本工具不是安全沙箱。",
            BUG_PROBE_NOTE + "。",
            "退出码 0 仅表示未发现 error，不构成可上车结论；实车验收另需安全全停前提与停障/断联实验。",
        ],
    }


def write_json_atomic(path, text):
    """同目录临时文件 + fsync + os.replace；失败时清理临时文件且不触碰原文件。"""
    directory = os.path.dirname(os.path.abspath(path)) or "."
    handle = None
    tmp_path = None
    try:
        fd, tmp_path = tempfile.mkstemp(prefix=".workspace_audit-", suffix=".tmp", dir=directory)
        handle = os.fdopen(fd, "w", encoding="utf-8", newline="\n")
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
        handle.close()
        handle = None
        os.replace(tmp_path, path)
        tmp_path = None
    finally:
        if handle is not None:
            handle.close()
        if tmp_path is not None:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass


def build_parser():
    parser = argparse.ArgumentParser(
        prog="workspace_audit.py", description="离线只读工作区审计（不导入/不执行被审计工程，仅标准库）"
    )
    parser.add_argument("--root", required=True, help="待审计工作区根目录（必须已存在）")
    parser.add_argument("--output", help="可选：JSON 输出文件；默认只打印到 stdout，指定后才写（原子写）")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    root = os.path.realpath(os.path.abspath(args.root))
    if not os.path.isdir(root):
        sys.stderr.write("错误: --root 不是已存在的目录: %s\n" % args.root)
        return 2
    if not os.access(root, os.R_OK | os.X_OK):
        sys.stderr.write("错误: --root 不可读或不可进入: %s\n" % root)
        return 2
    try:
        report = audit(root)
    except OSError as exc:
        sys.stderr.write("错误: 扫描失败: %s\n" % exc)
        return 2

    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        try:
            write_json_atomic(args.output, text)
        except OSError as exc:
            sys.stderr.write("错误: 写入 %s 失败: %s\n" % (args.output, exc))
            return 2
        sys.stderr.write("已写入 %s\n" % args.output)
    else:
        sys.stdout.write(text)

    summary = report["summary"]
    if summary["traversal_failures"]:
        # 遍历失败是 IO 失败：报告已输出作为证据，但整体不算完成。
        sys.stderr.write("错误: %d 处目录无法读取，扫描不完整\n" % summary["traversal_failures"])
        return 2
    return 0 if summary["errors"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
