#!/usr/bin/env python3
from __future__ import annotations

import datetime as dt
import os
import pathlib
import re
import subprocess


def git_output(args: list[str]) -> str:
    return subprocess.check_output(args, text=True).strip()


def git_output_or_empty(args: list[str]) -> str:
    try:
        return git_output(args)
    except Exception:
        return ""


def update_build_version(path: pathlib.Path, build_number: int) -> bool:
    new_content = f"{build_number}\n"
    if path.exists() and path.read_text(encoding="utf-8") == new_content:
        return False
    path.write_text(new_content, encoding="utf-8")
    return True


def ensure_changelog(path: pathlib.Path) -> None:
    if path.exists():
        return
    path.write_text("# Changelog\n", encoding="utf-8")


def get_repo_commit_base_url() -> str:
    remote = git_output_or_empty(["git", "config", "--get", "remote.origin.url"]).strip()
    if remote:
        if remote.endswith(".git"):
            remote = remote[:-4]

        patterns = [
            r"^github:(?P<slug>[\w.-]+/[\w.-]+)$",
            r"^git@github\.com:(?P<slug>[\w.-]+/[\w.-]+)$",
            r"^https://github\.com/(?P<slug>[\w.-]+/[\w.-]+)$",
            r"^https://[^@]+@github\.com/(?P<slug>[\w.-]+/[\w.-]+)$",
            r"^ssh://git@github\.com/(?P<slug>[\w.-]+/[\w.-]+)$",
        ]
        for pattern in patterns:
            match = re.match(pattern, remote)
            if match:
                return f"https://github.com/{match.group('slug')}/commit"

    repo_slug = os.environ.get("GITHUB_REPOSITORY", "").strip()
    if repo_slug:
        return f"https://github.com/{repo_slug}/commit"

    return ""


def load_previous_build_number(path: pathlib.Path) -> int:
    if not path.exists():
        return 0
    raw = path.read_text(encoding="utf-8").strip()
    if not raw:
        return 0
    try:
        return int(raw)
    except ValueError:
        return 0


def get_commits_for_release(previous_build: int, current_build: int) -> list[tuple[str, str]]:
    if previous_build <= 0 or current_build <= previous_build:
        sha = git_output(["git", "rev-parse", "HEAD"])
        subject = git_output(["git", "log", "-1", "--pretty=%s"])
        return [(sha, subject)]

    commit_count = current_build - previous_build
    raw = git_output(["git", "log", "--reverse", "-n", str(commit_count), "--pretty=%H%x1f%s", "HEAD"])
    commits: list[tuple[str, str]] = []
    for line in raw.splitlines():
        parts = line.split("\x1f", 1)
        if len(parts) != 2:
            continue
        sha, subject = parts[0].strip(), parts[1].strip()
        if not sha:
            continue
        if subject.startswith("chore(release):") and "[skip ci]" in subject:
            continue
        commits.append((sha, subject))

    if commits:
        return commits

    sha = git_output(["git", "rev-parse", "HEAD"])
    subject = git_output(["git", "log", "-1", "--pretty=%s"])
    return [(sha, subject)]


def format_commit_link(commit_base_url: str, sha: str) -> str:
    short_sha = sha[:7]
    if commit_base_url:
        return f"[{short_sha}]({commit_base_url}/{sha})"
    return short_sha


def remove_unreleased_section(lines: list[str]) -> bool:
    if "## Unreleased" not in lines:
        return False

    start = lines.index("## Unreleased")
    end = len(lines)
    for i in range(start + 1, len(lines)):
        if lines[i].startswith("## "):
            end = i
            break

    del lines[start:end]
    while start < len(lines) - 1 and lines[start].strip() == "" and lines[start + 1].strip() == "":
        del lines[start]
    return True


def remove_build_history_section(lines: list[str]) -> bool:
    if "## Build History" not in lines:
        return False

    start = lines.index("## Build History")
    end = len(lines)
    for i in range(start + 1, len(lines)):
        if lines[i].startswith("## "):
            end = i
            break

    del lines[start:end]
    while start < len(lines) - 1 and lines[start].strip() == "" and lines[start + 1].strip() == "":
        del lines[start]
    return True


def build_release_section(
    build_number: int,
    commits: list[tuple[str, str]],
    commit_base_url: str,
) -> list[str]:
    release_lines = [f"## v{build_number} - {dt.date.today().isoformat()}", ""]

    for sha, subject in commits:
        link = format_commit_link(commit_base_url, sha)
        release_lines.append(f"- {link} {subject}")

    release_lines.append("")
    return release_lines


def upsert_release_section(
    path: pathlib.Path,
    build_number: int,
    commits: list[tuple[str, str]],
    commit_base_url: str,
) -> bool:
    content = path.read_text(encoding="utf-8")
    lines = content.splitlines()

    changed = remove_unreleased_section(lines)
    changed = remove_build_history_section(lines) or changed
    if any(line.startswith(f"## v{build_number}") for line in lines):
        if changed:
            path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
        return changed

    release_lines = build_release_section(
        build_number=build_number,
        commits=commits,
        commit_base_url=commit_base_url,
    )

    insert_at = len(lines)
    if insert_at > 0 and lines[insert_at - 1] != "":
        lines.append("")
        insert_at += 1
    lines[insert_at:insert_at] = release_lines

    lines = [line.rstrip() for line in lines]
    text = "\n".join(lines).rstrip() + "\n"
    if text == content:
        return False
    path.write_text(text, encoding="utf-8")
    return True


def main() -> int:
    repo_root = pathlib.Path(__file__).resolve().parents[1]
    commit_base_url = get_repo_commit_base_url()

    build_file = repo_root / "BUILD_VERSION"
    previous_build_number = load_previous_build_number(build_file)

    build_number = int(git_output(["git", "rev-list", "--count", "HEAD"]))
    commits = get_commits_for_release(previous_build_number, build_number)

    changelog_file = repo_root / "CHANGELOG.md"

    changed = False
    ensure_changelog(changelog_file)
    changed = upsert_release_section(
        path=changelog_file,
        build_number=build_number,
        commits=commits,
        commit_base_url=commit_base_url,
    ) or changed
    changed = update_build_version(build_file, build_number) or changed

    return 0 if changed else 0


if __name__ == "__main__":
    raise SystemExit(main())
