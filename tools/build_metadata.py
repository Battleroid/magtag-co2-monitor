from __future__ import annotations

import pathlib
import subprocess
import time

Import("env")


def git_output(args: list[str]) -> str | None:
    try:
        return subprocess.check_output(args, text=True).strip()
    except Exception:
        return None


def git_is_dirty() -> bool:
    try:
        out = subprocess.check_output(["git", "status", "--porcelain"], text=True)
        return bool(out.strip())
    except Exception:
        return False


repo_root = pathlib.Path(env.subst("$PROJECT_DIR")).resolve()
build_version_file = repo_root / "BUILD_VERSION"

build_version = "0"
commit_count = git_output(["git", "rev-list", "--count", "HEAD"])
if commit_count:
    build_version = commit_count
elif build_version_file.exists():
    text = build_version_file.read_text(encoding="utf-8").strip()
    if text:
        build_version = text

build_hash = git_output(["git", "rev-parse", "--short", "HEAD"]) or "unknown"
build_epoch = int(time.time())
build_dirty = "IN_PROGRESS" if git_is_dirty() else "CLEAN"

env.Append(
    CPPDEFINES=[
        ("BUILD_VERSION_STR", env.StringifyMacro(build_version)),
        ("BUILD_HASH_STR", env.StringifyMacro(build_hash)),
        ("BUILD_EPOCH_UNIX", build_epoch),
        ("BUILD_DIRTY_STR", env.StringifyMacro(build_dirty)),
    ]
)
