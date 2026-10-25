# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""/identify_product over a real ROS graph: the node converts messages and wraps the service."""
# One process: the service node and a client share a context on a private DDS domain, so the
# test cannot hear (or be heard by) another run. The identity logic itself is covered without ROS
# in test_identify_service.py; this checks the message contract and the sidecar-absent answer.

import os
from pathlib import Path
import random
import sys
import threading

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rclpy.executors import SingleThreadedExecutor  # noqa: E402,I100,I101
from restocker_interfaces.srv import IdentifyProduct  # noqa: E402
from restocker_perception.identity import IdentityConfig, write_calibration  # noqa: E402
from restocker_perception.identity.benchmark import save_whitener  # noqa: E402
from restocker_perception.identity.node import IdentifyProductNode  # noqa: E402
from restocker_perception.registry import FakeEmbeddingBackend, Registry, Whitener  # noqa: E402
from restocker_perception.registry import render as rd  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

GTIN = "2000000000015"


def image_message(pixels: np.ndarray, encoding: str = "rgb8") -> Image:
    message = Image()
    message.height, message.width = pixels.shape[:2]
    message.encoding = encoding
    message.step = pixels.shape[1] * 3
    message.data = pixels.tobytes()
    return message


@pytest.fixture(scope="module")
def ros_context():
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=random.Random(os.getpid()).randint(120, 200))
    yield context
    rclpy.shutdown(context=context)


def call(context, node_factory, request):
    node = node_factory(context)
    client_node = rclpy.create_node("identify_client", context=context)
    client = client_node.create_client(IdentifyProduct, "identify_product")
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(client_node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        assert client.wait_for_service(timeout_sec=10.0)
        future = client.call_async(request)
        for _ in range(200):
            if future.done():
                break
            threading.Event().wait(0.05)
        return future.result()
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()


def test_unconfigured_node_answers_unknown(ros_context):
    request = IdentifyProduct.Request()
    request.crops = [image_message(np.zeros((32, 32, 3), np.uint8))]
    response = call(ros_context, lambda ctx: IdentifyProductNode(context=ctx), request)
    assert response.status == IdentifyProduct.Response.STATUS_UNKNOWN
    assert not response.sidecar_available and not response.has_sku and response.sku == ""


def test_configured_node_names_a_registered_crop(ros_context, tmp_path):
    registry = Registry.create(tmp_path / "reg")
    rng = np.random.default_rng(1)
    texture = np.tile(rng.uniform(0.1, 0.9, 3), (96, 192, 1)).astype(np.float32)
    texture[10:30, 20:80] = rng.uniform(0, 1, 3)
    registry.register(
        gtin=GTIN,
        display_name="P",
        product_class="can",
        geometry_key="can.t",
        radius_m=0.033,
        height_m=0.122,
        images=rd.render_views(texture, 0.033, 0.122),
    )
    registry.register(
        gtin="2000000000022",
        display_name="Q",
        product_class="can",
        geometry_key="can.q",
        radius_m=0.033,
        height_m=0.122,
        images=rd.render_views(np.flip(texture, axis=0).copy(), 0.033, 0.122),
    )
    backend = FakeEmbeddingBackend()
    pool = np.concatenate([backend.embed(registry.view_images(g)) for g in registry.product_ids])
    whitener = Whitener.fit(pool, 8)
    save_whitener(tmp_path / "w.npz", whitener)
    write_calibration(
        tmp_path / "cal.json",
        IdentityConfig(alone_min_score=0.0, alone_min_margin=0.0, alone_min_vote=1.0),
        {
            "backend": f"{backend.name} {backend.version}",
            "registry_sha256": registry.content_hash(),
            "whitening": {"fingerprint": whitener.fingerprint},
        },
    )

    def node(context):
        return IdentifyProductNode(
            context=context,
            parameter_overrides=[
                rclpy.parameter.Parameter("registry_path", value=str(tmp_path / "reg")),
                rclpy.parameter.Parameter("calibration_path", value=str(tmp_path / "cal.json")),
                rclpy.parameter.Parameter("whitener_path", value=str(tmp_path / "w.npz")),
                rclpy.parameter.Parameter("backend", value="fake"),
                rclpy.parameter.Parameter("ocr", value="none"),
            ],
        )

    request = IdentifyProduct.Request()
    request.crops = [image_message(registry.view_images(GTIN)[2], "rgb8")]
    response = call(ros_context, node, request)
    assert response.sidecar_available and response.has_sku and response.sku == GTIN
    assert response.status == IdentifyProduct.Response.STATUS_IDENTIFIED
    assert list(response.evidence_kinds) == ["embedding"]
    assert response.registry_sha256 == registry.content_hash()

    bad = IdentifyProduct.Request()
    bad.crops = [image_message(np.zeros((8, 8, 3), np.uint8), "mono8")]
    response = call(ros_context, node, bad)
    assert response.status == IdentifyProduct.Response.STATUS_ABSTAIN
    assert "bad request" in response.reason
