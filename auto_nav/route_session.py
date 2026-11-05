"""Session-scoped temporary route lifecycle.

Persistent routes live in ``online_slam_goals/<env>/routes`` and are map-frame
data with a map identity.  Temporary routes live under
``runtime/sessions/<session_id>/routes`` and are only valid for the process
that created them.
"""

from __future__ import annotations

import json
import os
import shutil
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

DEFAULT_SESSION_ROOT = Path(__file__).resolve().parents[1] / "runtime" / "sessions"
MANIFEST_NAME = "manifest.json"
ROUTES_DIR = "routes"
DIAGNOSTICS_DIR = "diagnostics"


@dataclass
class Session:
    session_id: str
    path: Path
    manifest: dict

    @property
    def routes_dir(self) -> Path:
        return self.path / ROUTES_DIR

    @property
    def diagnostics_dir(self) -> Path:
        return self.path / DIAGNOSTICS_DIR

    @property
    def active(self) -> bool:
        return self.manifest.get("status") == "active"


def session_root(root=None) -> Path:
    if root is not None:
        return Path(root).expanduser().resolve()
    env_root = os.environ.get("PFA_SESSION_ROOT")
    return Path(env_root).expanduser().resolve() if env_root else DEFAULT_SESSION_ROOT


def _write_manifest(path: Path, manifest: dict) -> None:
    tmp = path.with_suffix(".tmp")
    tmp.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    tmp.replace(path)


def _load_manifest(session_dir: Path) -> dict:
    manifest_path = session_dir / MANIFEST_NAME
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无效的 session manifest: {manifest_path}: {exc}") from exc
    if not isinstance(data, dict) or not data.get("session_id"):
        raise ValueError(f"session manifest 缺少 session_id: {manifest_path}")
    return data


def create_session(kind="navigation", root=None, session_id=None) -> Session:
    root = session_root(root)
    session_id = session_id or uuid.uuid4().hex
    if Path(session_id).name != session_id or not session_id:
        raise ValueError("session_id 必须是单段目录名")
    path = root / session_id
    if path.exists():
        raise FileExistsError(f"session 已存在: {path}")
    (path / ROUTES_DIR).mkdir(parents=True, exist_ok=False)
    (path / DIAGNOSTICS_DIR).mkdir(parents=True, exist_ok=True)
    manifest = {
        "version": 1,
        "session_id": session_id,
        "kind": str(kind),
        "pid": os.getpid(),
        "created_at": time.time(),
        "updated_at": time.time(),
        "status": "active",
    }
    _write_manifest(path / MANIFEST_NAME, manifest)
    return Session(session_id, path, manifest)


def load_session(session_id, root=None):
    if not session_id or Path(str(session_id)).name != str(session_id):
        raise ValueError("session_id 必须是单段目录名")
    path = session_root(root) / str(session_id)
    if not path.is_dir():
        return None
    manifest = _load_manifest(path)
    if manifest.get("session_id") != session_id:
        raise ValueError("session manifest 与目录名不一致")
    return Session(session_id, path, manifest)


def validate_session(session_id, root=None) -> Session:
    if not session_id:
        raise ValueError("缺少 session_id")
    session = load_session(session_id, root=root)
    if session is None:
        raise ValueError(f"session 不存在: {session_id}")
    if not session.active:
        raise ValueError(f"session 已结束，不能复用: {session_id}")
    return session


def is_session_route(path, root=None) -> bool:
    root = session_root(root)
    try:
        relative = Path(path).expanduser().resolve().relative_to(root)
    except ValueError:
        return False
    parts = relative.parts
    return len(parts) >= 3 and parts[1] == ROUTES_DIR


def validate_session_route(path, session_id, root=None) -> Session:
    """Validate that ``path`` belongs to the active ``session_id`` routes dir."""
    if not is_session_route(path, root=root):
        raise ValueError(f"不是 session 临时路线: {path}")
    relative = Path(path).expanduser().resolve().relative_to(session_root(root))
    if relative.parts[0] != session_id:
        raise ValueError(
            f"路线属于 session {relative.parts[0]}，不能由 session {session_id} 使用"
        )
    session = validate_session(session_id, root=root)
    if not Path(path).is_file():
        raise ValueError(f"临时路线文件不存在: {path}")
    return session


def mark_status(session: Session, status, message=None) -> Session:
    session.manifest["status"] = str(status)
    session.manifest["updated_at"] = time.time()
    if message is not None:
        session.manifest["message"] = str(message)
    _write_manifest(session.path / MANIFEST_NAME, session.manifest)
    return session


def write_failure_diagnostics(session: Session, diagnostics) -> Path:
    session.diagnostics_dir.mkdir(parents=True, exist_ok=True)
    target = session.diagnostics_dir / "failure.json"
    target.write_text(
        json.dumps(diagnostics, ensure_ascii=False, indent=2, default=str) + "\n",
        encoding="utf-8",
    )
    return target


def finish_session(session: Session, success=True, diagnostics=None) -> None:
    """Delete executable temporary routes; keep diagnostics on failure."""
    if success:
        shutil.rmtree(session.path, ignore_errors=True)
        return
    failure_path = session.diagnostics_dir / "failure.json"
    if diagnostics is not None:
        write_failure_diagnostics(session, diagnostics)
    elif not failure_path.is_file():
        write_failure_diagnostics(
            session, {"session_id": session.session_id, "status": "failed"}
        )
    routes_dir = session.routes_dir
    if routes_dir.is_dir():
        for child in routes_dir.iterdir():
            if child.is_dir():
                shutil.rmtree(child, ignore_errors=True)
            else:
                child.unlink(missing_ok=True)
    mark_status(session, "failed")


def _pid_alive(pid) -> bool:
    try:
        pid = int(pid)
    except (TypeError, ValueError):
        return False
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def cleanup_stale_sessions(root=None):
    """Clean routes of sessions whose owner process is gone.

    Returns the session ids that were cleaned.  Diagnostics are retained; only
    executable routes are removed.  Failed sessions are left for an explicit
    operator cleanup so evidence is not silently destroyed.
    """
    root = session_root(root)
    if not root.is_dir():
        return []
    cleaned = []
    now = time.time()
    for path in sorted(root.iterdir()):
        if not path.is_dir():
            continue
        try:
            manifest = _load_manifest(path)
        except ValueError:
            continue
        if manifest.get("status") != "active":
            continue
        if _pid_alive(manifest.get("pid")):
            continue
        routes_dir = path / ROUTES_DIR
        if routes_dir.is_dir():
            for child in routes_dir.iterdir():
                if child.is_dir():
                    shutil.rmtree(child, ignore_errors=True)
                else:
                    child.unlink(missing_ok=True)
        manifest["status"] = "stale"
        manifest["updated_at"] = now
        _write_manifest(path / MANIFEST_NAME, manifest)
        cleaned.append(manifest["session_id"])
    return cleaned
