# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Static wiring contract for the explicit dense sensor validation gate (Card 094, CMB-SPEC-14).

Everything is read from the source tree: no simulator, no lease, no ROS graph. The test pins
the four wires that keep `test_dense_sensor_driven_runtime` out of every default suite while
keeping it runnable by name: the `dense-gate` just recipe, the label exclusions in both default
host suites and the Nix check, the CMake registration (both labels, no DISABLED, the renderer
resource lock kept), and the dense test's own identity tolerance, which no gate may relax.
"""

import os
from pathlib import Path
import re


def _environment_path(name: str) -> Path:
    """Read a source-tree path from the environment ctest sets for this test."""
    try:
        return Path(os.environ[name])
    except KeyError as error:
        raise RuntimeError(
            f"{name} is not set. ament_add_pytest_test provides it under ctest; run this "
            "file as `ctest -R test_dense_gate_wiring` (or pass RESTOCKER_BRINGUP_SOURCE_DIR "
            "and RESTOCKER_REPO_ROOT by hand), not as bare pytest"
        ) from error


BRINGUP = _environment_path("RESTOCKER_BRINGUP_SOURCE_DIR")
REPO_ROOT = _environment_path("RESTOCKER_REPO_ROOT")

JUSTFILE = REPO_ROOT / "justfile"
FLAKE = REPO_ROOT / "flake.nix"
CMAKE = BRINGUP / "CMakeLists.txt"
DENSE_TEST = BRINGUP / "test" / "test_dense_sensor_driven_runtime.py"


def _recipe_body(name: str) -> str:
    """Return the indented body of just recipe `name`, or '' when the recipe is absent."""
    lines = JUSTFILE.read_text(encoding="utf-8").splitlines()
    header = re.compile(rf"^{re.escape(name)}(?:\s[^:]*)?:")
    body: list[str] = []
    capturing = False
    for line in lines:
        if not capturing:
            if header.match(line):
                capturing = True
            continue
        if line and not line[0].isspace():
            break
        body.append(line)
    return "\n".join(body)


def test_dense_gate_recipe_runs_the_dense_test_in_one_simulator_slot() -> None:
    """The named gate must select exactly this target, serialized and with output on failure."""
    body = _recipe_body("dense-gate")
    assert body, "justfile has no `dense-gate` recipe"
    assert "test_dense_sensor_driven_runtime" in body, "the gate must select the dense test"
    assert "-R '^test_dense_sensor_driven_runtime$'" in body, "the gate must select only it"
    assert "with_simulator_slot.bash" in body, "the gate must hold a machine-wide simulator slot"
    assert "RESTOCKER_SIM_SLOTS=1" in body, "the gate must be predeclared to one slot"
    assert "with_workspace.bash" in body, "the gate must run inside the built workspace env"
    assert "-j1" in body, "the gate is predeclared to a single ctest job"
    assert "--output-on-failure" in body, "a red gate must name its failing case"
    assert "--no-tests=error" in body, "a gate that matches no test must not report green"
    assert "load1" in body, "the gate must check and record the quiet-start load"


def test_default_host_suites_exclude_the_dense_gate_label() -> None:
    """`just test` and `just test-package` are default suites: both must deselect dense-gate."""
    for name in ("test", "test-package"):
        body = _recipe_body(name)
        assert body, f"justfile has no `{name}` recipe"
        assert "--label-exclude dense-gate" in body, (
            f"`just {name}` must pass --label-exclude dense-gate so a green default suite "
            "never implies dense coverage"
        )


def test_nix_check_excludes_the_dense_gate_label_too() -> None:
    """The Nix selection must exclude dense-gate by name, not only via the renderer label."""
    flake = FLAKE.read_text(encoding="utf-8")
    match = re.search(r"--label-exclude\s+'([^']+)'", flake)
    assert match, "flake.nix must pass a quoted --label-exclude to colcon test"
    excluded = match.group(1).split("|")
    assert "renderer" in excluded, "the Nix check must keep excluding the renderer label"
    assert "dense-gate" in excluded, "the Nix check must exclude the dense-gate label by name"


def test_cmake_registration_is_enabled_labelled_and_locked() -> None:
    """Registered, both labels, no DISABLED, renderer resource lock kept (CMB-SPEC-14 §2)."""
    cmake = CMAKE.read_text(encoding="utf-8")
    assert re.search(
        r"add_launch_test\(\s*test/test_dense_sensor_driven_runtime\.py\s*"
        r"TARGET test_dense_sensor_driven_runtime",
        cmake,
    ), "the dense test must stay registered"
    disabled = re.search(
        r"set_tests_properties\(\s*test_dense_sensor_driven_runtime[^)]*DISABLED",
        cmake,
        re.DOTALL,
    )
    assert disabled is None, "the dense test must not be DISABLED: labels are the exclusion"
    assert re.search(
        r"set_property\(TEST test_dense_sensor_driven_runtime\s+APPEND PROPERTY LABELS"
        r' "dense-gate"\)',
        cmake,
    ), "the dense test must carry the dense-gate label"
    assert re.search(
        r"set_property\(TEST[^)]*test_dense_sensor_driven_runtime[^)]*"
        r'LABELS "renderer"\)',
        cmake,
        re.DOTALL,
    ), "the dense test must keep the renderer label (Nix sandbox exclusion)"
    assert re.search(
        r"set_property\(TEST[^)]*test_dense_sensor_driven_runtime[^)]*"
        r'RESOURCE_LOCK "restocker-renderer-simulator"\)',
        cmake,
        re.DOTALL,
    ), "the dense test must keep the renderer simulator resource lock"


def test_dense_test_assertions_are_not_weakened() -> None:
    """The gate runs the test's own contract: identity bound and front-full stay as specified."""
    source = DENSE_TEST.read_text(encoding="utf-8")
    assert "IDENTITY_MISMATCH_M = 0.05" in source, "the 0.05 m identity bound must not move"
    assert "PHASE_FRONT_FULL" in source, "the front-full requirement must not move"
    assert "total_front_deficit" in source, "the zero-deficit assertion must not move"
