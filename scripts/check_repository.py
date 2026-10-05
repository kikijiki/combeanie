#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Validate repository structure and ROS package dependency policy."""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / "ros_ws" / "src"

EXPECTED_ROOT_FILES = {
    ".clang-format",
    ".envrc",
    ".gitignore",
    "AGENTS.md",
    "CONTRIBUTING.md",
    "LICENSE",
    "README.md",
    "flake.lock",
    "flake.nix",
    "justfile",
    "pyproject.toml",
}

EXPECTED_PACKAGES = {
    "restocker_benchmarks",
    "restocker_bringup",
    "restocker_control",
    "restocker_description",
    "restocker_gazebo",
    "restocker_interfaces",
    "restocker_moveit_config",
    "restocker_perception",
    "restocker_reasoner",
    "restocker_recovery",
    "restocker_task_executor",
    "restocker_world_state",
}

DEPENDENCY_TAGS = {
    "build_depend",
    "build_export_depend",
    "depend",
    "exec_depend",
    "test_depend",
}


def report_error(message: str, errors: list[str]) -> None:
    errors.append(message)


def validate_root(errors: list[str]) -> None:
    for relative_path in sorted(EXPECTED_ROOT_FILES):
        if not (REPOSITORY_ROOT / relative_path).is_file():
            report_error(f"missing required root file: {relative_path}", errors)

    if (REPOSITORY_ROOT / "Makefile").exists():
        report_error("top-level Makefile is forbidden; use justfile", errors)

    envrc = (REPOSITORY_ROOT / ".envrc").read_text(encoding="utf-8")
    if envrc != "use flake\n":
        report_error(".envrc must contain exactly 'use flake'", errors)

    ignored = (REPOSITORY_ROOT / ".gitignore").read_text(encoding="utf-8")
    for path in ("/ros_ws/build/", "/ros_ws/install/", "/ros_ws/log/"):
        if path not in ignored:
            report_error(f".gitignore does not exclude {path}", errors)


def parse_packages(errors: list[str]) -> dict[str, set[str]]:
    if not SOURCE_ROOT.is_dir():
        report_error("missing ros_ws/src", errors)
        return {}

    package_dirs = {
        path.name: path
        for path in SOURCE_ROOT.iterdir()
        if path.is_dir() and not path.name.startswith(".")
    }
    actual_packages = set(package_dirs)

    for missing in sorted(EXPECTED_PACKAGES - actual_packages):
        report_error(f"missing ROS package directory: {missing}", errors)
    for unexpected in sorted(actual_packages - EXPECTED_PACKAGES):
        report_error(f"unexpected ROS package directory: {unexpected}", errors)

    dependencies: dict[str, set[str]] = defaultdict(set)
    for directory_name, package_dir in sorted(package_dirs.items()):
        manifest_path = package_dir / "package.xml"
        cmake_path = package_dir / "CMakeLists.txt"
        readme_path = package_dir / "README.md"
        for required_path in (manifest_path, cmake_path, readme_path):
            if not required_path.is_file():
                report_error(f"{directory_name}: missing {required_path.name}", errors)

        if not manifest_path.is_file():
            continue

        try:
            root = ET.parse(manifest_path).getroot()
        except ET.ParseError as error:
            report_error(f"{directory_name}: invalid package.xml: {error}", errors)
            continue

        manifest_name = (root.findtext("name") or "").strip()
        if manifest_name != directory_name:
            report_error(
                f"{directory_name}: manifest name is '{manifest_name}', "
                f"expected '{directory_name}'",
                errors,
            )

        if root.attrib.get("format") != "3":
            report_error(f"{directory_name}: package.xml must use format 3", errors)

        licenses = {(element.text or "").strip() for element in root.findall("license")}
        if "MIT" not in licenses:
            report_error(f"{directory_name}: MIT license is not declared", errors)

        build_type = root.find("./export/build_type")
        if build_type is None or (build_type.text or "").strip() != "ament_cmake":
            report_error(f"{directory_name}: export/build_type must be ament_cmake", errors)

        for child in root:
            if child.tag in DEPENDENCY_TAGS and child.text:
                dependency = child.text.strip()
                if dependency in EXPECTED_PACKAGES:
                    dependencies[directory_name].add(dependency)

    return dependencies


def validate_dependency_policy(dependencies: dict[str, set[str]], errors: list[str]) -> None:
    if dependencies.get("restocker_interfaces"):
        report_error("restocker_interfaces must not depend on application packages", errors)

    for package, package_dependencies in sorted(dependencies.items()):
        if package != "restocker_bringup" and "restocker_bringup" in package_dependencies:
            report_error(
                f"{package}: runtime packages must not depend on restocker_bringup", errors
            )

    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(package: str, trail: tuple[str, ...]) -> None:
        if package in visiting:
            cycle_start = trail.index(package)
            cycle = (*trail[cycle_start:], package)
            report_error(f"application package dependency cycle: {' -> '.join(cycle)}", errors)
            return
        if package in visited:
            return

        visiting.add(package)
        for dependency in sorted(dependencies.get(package, set())):
            visit(dependency, (*trail, package))
        visiting.remove(package)
        visited.add(package)

    for package in sorted(EXPECTED_PACKAGES):
        visit(package, ())


def main() -> int:
    errors: list[str] = []
    validate_root(errors)
    dependencies = parse_packages(errors)
    validate_dependency_policy(dependencies, errors)

    if errors:
        for error in errors:
            print(f"error: {error}", file=sys.stderr)
        print(f"repository check failed with {len(errors)} error(s)", file=sys.stderr)
        return 1

    edge_count = sum(len(items) for items in dependencies.values())
    print(
        f"repository check passed: {len(EXPECTED_PACKAGES)} packages, "
        f"{edge_count} internal dependency edges"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
