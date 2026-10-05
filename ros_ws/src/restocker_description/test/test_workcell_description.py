# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static geometry and frame tests for the reusable workcell assembly."""

import math
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
import yaml

SOURCE_DIR = Path(os.environ["RESTOCKER_DESCRIPTION_SOURCE_DIR"])
WORKCELL_XACRO = SOURCE_DIR / "urdf" / "workcell.urdf.xacro"
GEOMETRY_FILE = SOURCE_DIR / "config" / "workcell_geometry.yaml"
CATALOG_FILE = SOURCE_DIR / "config" / "product_collision_catalog.yaml"


@pytest.fixture(scope="module")
def geometry() -> dict:
    return yaml.safe_load(GEOMETRY_FILE.read_text())


@pytest.fixture(scope="module")
def catalog() -> dict:
    return yaml.safe_load(CATALOG_FILE.read_text())


@pytest.fixture(scope="module")
def workcell() -> ET.Element:
    expanded = subprocess.run(
        ["xacro", str(WORKCELL_XACRO)], check=True, capture_output=True, text=True
    ).stdout
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf:
        urdf.write(expanded)
        urdf.flush()
        subprocess.run(["check_urdf", urdf.name], check=True, capture_output=True, text=True)
    return ET.fromstring(expanded)


def _xyz(joint: ET.Element) -> tuple[float, float, float]:
    return tuple(float(value) for value in joint.find("origin").attrib["xyz"].split())


def _vector(value: str) -> tuple[float, float, float]:
    parsed = tuple(float(component) for component in value.split())
    assert len(parsed) == 3
    return parsed


def test_workcell_is_an_independent_shelf_root(workcell: ET.Element) -> None:
    links = {link.attrib["name"] for link in workcell.findall("link")}
    child_links = {joint.find("child").attrib["link"] for joint in workcell.findall("joint")}
    assert links - child_links == {"shelf"}
    assert workcell.findtext("gazebo/static") == "true"


def test_every_configured_lane_has_its_own_frame(workcell: ET.Element, geometry: dict) -> None:
    # Placement resolves its destination lane through tf2. The link set is compared as a whole so
    # a frame without a matching lane also fails.
    links = {link.attrib["name"] for link in workcell.findall("link")}
    assert links == set(geometry["lanes"]) | {
        "shelf",
        "roller_bed",
        "overhead_camera_link",
        "overhead_camera_optical_frame",
    }
    assert len(geometry["lanes"]) == 6


def test_lane_frames_start_at_rear_and_advance_along_positive_y(
    workcell: ET.Element, geometry: dict
) -> None:
    joints = {joint.attrib["name"]: joint for joint in workcell.findall("joint")}
    for lane_id, lane in geometry["lanes"].items():
        joint = joints[f"{lane_id}_joint"]
        assert _xyz(joint) == (lane["center_x_m"], 0.0, 0.0)
        assert joint.find("origin").attrib["rpy"] == "0 0 0"
        assert lane["frame_id"] == lane_id
        assert tuple(lane["insertion_axis"]) == (0.0, 1.0, 0.0)


def _bed_height(geometry: dict, depth: float) -> float:
    """Return the roller surface height above the shelf datum at a lane depth."""
    shelf = geometry["shelf"]
    datum = shelf["depth_m"] - shelf["front_retainer_depth_m"]
    return (datum - depth) * math.tan(math.radians(shelf["lane_incline_deg"]))


def test_roller_bed_is_a_separate_inclined_link(workcell: ET.Element, geometry: dict) -> None:
    # The bed is the only low-friction surface and URDF <gazebo reference> friction is per link,
    # so it cannot share the shelf link with the stock tray.
    bed = workcell.find("link[@name='roller_bed']")
    assert bed is not None
    joint = workcell.find("joint[@name='roller_bed_joint']")
    assert joint.find("parent").attrib["link"] == "shelf"
    assert _xyz(joint) == (0.0, 0.0, 0.0)
    assert joint.find("origin").attrib["rpy"] == "0 0 0"

    shelf_geometry = geometry["shelf"]
    incline = math.radians(shelf_geometry["lane_incline_deg"])
    collisions = {item.attrib["name"]: item for item in bed.findall("collision")}
    assert set(collisions) == {"roller_bed_collision"}
    slab = collisions["roller_bed_collision"]
    # As long as the shelf measured along the slope, and pitched nose-down toward the customer.
    assert _vector(slab.find("geometry/box").attrib["size"]) == pytest.approx(
        (
            shelf_geometry["width_m"],
            shelf_geometry["depth_m"] / math.cos(incline),
            shelf_geometry["support_thickness_m"],
        )
    )
    assert _vector(slab.find("origin").attrib["rpy"]) == pytest.approx((-incline, 0.0, 0.0))

    # The slab's top face is the surface every height in the survey is quoted against: level with
    # the lane floor at the rail face and rising behind it.
    centre = _vector(slab.find("origin").attrib["xyz"])
    half = shelf_geometry["support_thickness_m"] / 2.0
    surface_y = centre[1] + half * math.sin(incline)
    surface_z = centre[2] + half * math.cos(incline)
    assert surface_z == pytest.approx(_bed_height(geometry, surface_y))
    assert surface_y == pytest.approx(shelf_geometry["depth_m"] / 2.0)

    # The rollers are visual only, tangent to that surface, and spaced as surveyed.
    assert not bed.findall("collision[@name='roller_00_collision']")
    rollers = [
        item for item in bed.findall("visual") if item.attrib["name"] not in {"roller_bed_visual"}
    ]
    assert len(rollers) == int(shelf_geometry["depth_m"] / shelf_geometry["roller_pitch_m"])
    radius = shelf_geometry["roller_radius_m"]
    for index, roller in enumerate(rollers):
        cylinder = roller.find("geometry/cylinder")
        assert float(cylinder.attrib["radius"]) == radius
        assert float(cylinder.attrib["length"]) == shelf_geometry["width_m"]
        depth = (index + 0.5) * shelf_geometry["roller_pitch_m"]
        assert _vector(roller.find("origin").attrib["xyz"]) == pytest.approx(
            (
                0.0,
                depth - radius * math.sin(incline),
                _bed_height(geometry, depth) - radius * math.cos(incline),
            )
        )


def test_shelf_and_stock_tray_collision_envelopes(workcell: ET.Element, geometry: dict) -> None:
    shelf = workcell.find("link[@name='shelf']")
    collisions = {item.attrib["name"]: item for item in shelf.findall("collision")}
    shelf_geometry = geometry["shelf"]
    stock_geometry = geometry["stock_tray"]
    stock_tray = collisions["stock_tray_collision"]
    assert _vector(stock_tray.find("geometry/box").attrib["size"]) == (
        stock_geometry["width_m"],
        stock_geometry["depth_m"],
        stock_geometry["thickness_m"],
    )
    assert _vector(stock_tray.find("origin").attrib["xyz"]) == tuple(
        stock_geometry["center_xyz_m"]
    )
    assert {name for name in collisions if name.endswith("_collision")} == {
        f"{name}_collision" for name in shelf_geometry["divider_x_m"]
    } | {
        "front_retainer_collision",
        "stock_tray_collision",
    }


def test_lane_and_stock_volumes_fit_surveyed_collision_boundaries(
    workcell: ET.Element, geometry: dict, catalog: dict
) -> None:
    shelf = workcell.find("link[@name='shelf']")
    collisions = {item.attrib["name"]: item for item in shelf.findall("collision")}
    shelf_geometry = geometry["shelf"]
    # Sorted by position so out-of-order divider entries are measured against the lane they bound.
    divider_centers = sorted(
        _vector(collisions[f"{name}_collision"].find("origin").attrib["xyz"])[0]
        for name in shelf_geometry["divider_x_m"]
    )
    assert len(divider_centers) == len(geometry["lanes"]) + 1
    divider_half_width = shelf_geometry["divider_thickness_m"] / 2.0
    # On a gravity lane the rail face lies inside the usable volume: products rest leaning on it,
    # and a volume stopping short of it would report completed placements as obstructions.
    rail_face_depth = shelf_geometry["depth_m"] - shelf_geometry["front_retainer_depth_m"]
    incline = math.tan(math.radians(shelf_geometry["lane_incline_deg"]))
    # How far past the rail face the tallest product leans: it touches the rail at the rail's top
    # edge and everything above that overhangs it.
    tallest = max(item["shape"]["height_m"] for item in catalog["geometries"])
    assert tallest > shelf_geometry["front_retainer_height_m"]
    lean = (tallest - shelf_geometry["front_retainer_height_m"]) * incline
    assert divider_centers[0] == pytest.approx(-shelf_geometry["width_m"] / 2.0)
    assert divider_centers[-1] == pytest.approx(shelf_geometry["width_m"] / 2.0)

    for index, (lane_id, lane) in enumerate(sorted(geometry["lanes"].items())):
        left_inner = divider_centers[index] + divider_half_width
        right_inner = divider_centers[index + 1] - divider_half_width
        assert lane["center_x_m"] == pytest.approx((left_inner + right_inner) / 2.0)
        assert lane["usable_width_m"] + 2.0 * lane["side_clearance_m"] <= (
            right_inner - left_inner + 1.0e-12
        )
        lane_front = lane["rear_clearance_m"] + lane["usable_depth_m"]
        # The volume stops inside the shelf, with its surveyed margin to the front edge.
        assert lane_front + lane["front_clearance_m"] <= (shelf_geometry["depth_m"] + 1.0e-12)
        # It also reaches far enough past the rail face to hold a product leaning on the rail.
        assert rail_face_depth < lane_front
        assert rail_face_depth + lean <= lane_front + 1.0e-12
        # The ceiling clears the bed's rear lip plus the tallest product, where the arm sets it
        # down.
        assert lane["floor_clearance_m"] + lane["usable_height_m"] <= (
            shelf_geometry["divider_height_m"] + 1.0e-12
        )
        assert _bed_height(geometry, 0.0) + tallest <= (
            lane["floor_clearance_m"] + lane["usable_height_m"] + 1.0e-12
        )
        # Release depth is rear clearance + product radius + entry clearance, so it is checked per
        # catalogued product: each must be contained when released and released behind the rail
        # face so the bed carries it forward.
        for item in catalog["geometries"]:
            release = (
                lane["rear_clearance_m"]
                + item["shape"]["radius_m"]
                + lane["insert_entry_clearance_m"]
            )
            assert release - item["shape"]["radius_m"] >= lane["rear_clearance_m"]
            assert release < rail_face_depth
        assert lane["frame_id"] == lane_id

    stock = geometry["stock_tray"]
    volume = stock["usable_volume"]
    assert volume["size_xyz_m"][0] <= stock["width_m"]
    assert volume["size_xyz_m"][1] <= stock["depth_m"]
    tray_top = stock["center_xyz_m"][2] + stock["thickness_m"] / 2.0
    volume_floor = volume["center_xyz_m"][2] - volume["size_xyz_m"][2] / 2.0
    assert volume_floor == pytest.approx(tray_top)


def test_versioned_geometry_file_is_the_actual_xacro_input(geometry: dict) -> None:
    assert geometry["schema_version"] == 1
    modified = yaml.safe_load(GEOMETRY_FILE.read_text())
    modified["shelf"]["width_m"] = 2.6
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as config:
        yaml.safe_dump(modified, config)
        config.flush()
        expanded = subprocess.run(
            ["xacro", str(WORKCELL_XACRO), f"geometry_config:={config.name}"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    workcell = ET.fromstring(expanded)
    slab = workcell.find("link[@name='roller_bed']/collision[@name='roller_bed_collision']")
    assert _vector(slab.find("geometry/box").attrib["size"])[0] == 2.6


def test_overhead_camera_uses_ros_optical_axes(workcell: ET.Element) -> None:
    joint = workcell.find("joint[@name='overhead_camera_optical_joint']")
    roll, pitch, yaw = (float(value) for value in joint.find("origin").attrib["rpy"].split())
    assert roll == pytest.approx(-math.pi / 2.0)
    assert pitch == 0.0
    assert yaw == pytest.approx(-math.pi / 2.0)


def _rotation(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    """Return the URDF rpy rotation Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _joint_pose(joint: ET.Element) -> tuple[list[list[float]], tuple[float, float, float]]:
    origin = joint.find("origin")
    return (
        _rotation(*(float(value) for value in origin.attrib["rpy"].split())),
        tuple(float(value) for value in origin.attrib["xyz"].split()),
    )


def _shelf_from_optical(workcell: ET.Element) -> tuple[list[list[float]], tuple[float, ...]]:
    """Compose shelf -> overhead_camera_link -> overhead_camera_optical_frame."""
    mount_rotation, mount_translation = _joint_pose(
        workcell.find("joint[@name='overhead_camera_mount_joint']")
    )
    optical_rotation, optical_translation = _joint_pose(
        workcell.find("joint[@name='overhead_camera_optical_joint']")
    )
    rotation = [
        [sum(mount_rotation[i][k] * optical_rotation[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]
    translation = tuple(
        mount_translation[i] + sum(mount_rotation[i][k] * optical_translation[k] for k in range(3))
        for i in range(3)
    )
    return rotation, translation


def _overhead_sensor(workcell: ET.Element) -> ET.Element:
    return workcell.find("gazebo[@reference='overhead_camera_link']/sensor")


def test_overhead_camera_publishes_in_its_optical_frame(workcell: ET.Element) -> None:
    # Gazebo would otherwise stamp images with the sensor's scoped name, which is not a TF frame.
    sensor = _overhead_sensor(workcell)
    assert sensor is not None
    assert sensor.attrib["type"] == "rgbd_camera"
    assert sensor.findtext("gz_frame_id") == "overhead_camera_optical_frame"
    assert sensor.findtext("topic") == "overhead_camera"
    assert float(sensor.findtext("update_rate")) > 0.0
    # The sensor renders from the link origin, which is the optical frame, so they cannot drift.
    assert sensor.find("pose") is None
    assert _joint_pose(workcell.find("joint[@name='overhead_camera_optical_joint']"))[1] == (
        0.0,
        0.0,
        0.0,
    )


def test_overhead_camera_frustum_contains_every_surveyed_work_region(
    workcell: ET.Element, geometry: dict
) -> None:
    """Every volume the robot works in must be inside the detection camera's image."""
    # Placement and FOV go together. Both work regions are surveyed in the shelf frame, which the
    # camera hangs off, so no simulator or scenario pose is needed.
    sensor = _overhead_sensor(workcell)
    camera = sensor.find("camera")
    width = int(camera.findtext("image/width"))
    height = int(camera.findtext("image/height"))
    focal_length = (width / 2.0) / math.tan(float(camera.findtext("horizontal_fov")) / 2.0)
    near = float(camera.findtext("clip/near"))
    far = float(camera.findtext("clip/far"))
    rotation, translation = _shelf_from_optical(workcell)

    def project(point: tuple[float, float, float]) -> tuple[float, float, float]:
        offset = [point[i] - translation[i] for i in range(3)]
        # The rotation is shelf <- optical, so its transpose maps a shelf point into the camera.
        camera_point = [sum(rotation[k][i] * offset[k] for k in range(3)) for i in range(3)]
        assert camera_point[2] > 0.0, f"{point} is behind the camera"
        return (
            focal_length * camera_point[0] / camera_point[2] + width / 2.0,
            focal_length * camera_point[1] / camera_point[2] + height / 2.0,
            camera_point[2],
        )

    regions = {}
    volume = geometry["stock_tray"]["usable_volume"]
    centre, size = volume["center_xyz_m"], volume["size_xyz_m"]
    regions["stock_tray"] = [
        (
            centre[0] + sx * size[0] / 2.0,
            centre[1] + sy * size[1] / 2.0,
            centre[2] + sz * size[2] / 2.0,
        )
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]
    for lane_id, lane in geometry["lanes"].items():
        half_width = lane["usable_width_m"] / 2.0
        regions[lane_id] = [
            (lane["center_x_m"] + sx * half_width, y, z)
            for sx in (-1, 1)
            for y in (
                lane["rear_clearance_m"],
                lane["rear_clearance_m"] + lane["usable_depth_m"],
            )
            for z in (
                lane["floor_clearance_m"],
                lane["floor_clearance_m"] + lane["usable_height_m"],
            )
        ]

    for name, corners in regions.items():
        for corner in corners:
            u, v, depth = project(corner)
            assert 0.0 <= u <= width, f"{name} corner {corner} projects to u={u:.1f}"
            assert 0.0 <= v <= height, f"{name} corner {corner} projects to v={v:.1f}"
            assert near < depth < far, f"{name} corner {corner} is {depth:.2f} m away"


def test_prefix_applies_to_every_workcell_link_and_joint() -> None:
    expanded = subprocess.run(
        ["xacro", str(WORKCELL_XACRO), "prefix:=test_"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    workcell = ET.fromstring(expanded)
    assert all(link.attrib["name"].startswith("test_") for link in workcell.findall("link"))
    assert all(joint.attrib["name"].startswith("test_") for joint in workcell.findall("joint"))
