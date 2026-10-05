# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Write the ROS parameter file test_race_planning_scene loads (Card 068).

The node gets exactly what move_group and the race-hosting ports get: the pinned MoveIt
configuration, the race's ompl + ompl_fallback namespaces, and move_group's robot padding read
from the move_group launch file itself, so a retuned padding cannot leave the test behind.

Usage: race_scene_parameters.py <move_group.launch.py> <output.yaml>
"""

from pathlib import Path
import re
import sys

from restocker_moveit_config import build_moveit_config, parallel_planning_parameters
import yaml


class _NoAliasDumper(yaml.SafeDumper):
    """rcl's parameter parser refuses YAML aliases, which shared lists would produce."""

    def ignore_aliases(self, data):
        return True


def _flatten(data, prefix=""):
    flat = {}
    for key, value in data.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(_flatten(value, name + "."))
        else:
            flat[name] = value
    return flat


def main(argv):
    """Write the parameter file; the move_group padding must be declared exactly once."""
    if len(argv) != 3:
        raise SystemExit(__doc__)
    launch_source = Path(argv[1]).read_text(encoding="utf-8")
    declared = re.findall(
        r'"robot_description_planning\.default_robot_padding"\s*:\s*([0-9.eE+-]+)', launch_source
    )
    if len(declared) != 1:
        raise SystemExit(f"expected one default_robot_padding in {argv[1]}, found {declared}")
    parameters = _flatten(build_moveit_config().to_dict())
    parameters.update(parallel_planning_parameters())
    parameters["robot_description_planning.default_robot_padding"] = float(declared[0])
    with open(argv[2], "w", encoding="utf-8") as stream:
        yaml.dump({"/**": {"ros__parameters": parameters}}, stream, Dumper=_NoAliasDumper)


if __name__ == "__main__":
    main(sys.argv)
