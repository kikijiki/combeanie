# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""/identify_object over a real ROS graph: the node converts messages and wraps the service."""
# One process, a private DDS domain (as test_identify_product_node.py). The logic is covered
# without ROS in test_identify_object.py; this checks the message contract and the
# sidecar-absent answer.

import math
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
from restocker_interfaces.srv import IdentifyObject  # noqa: E402
from restocker_perception.identity.ladder_node import IdentifyObjectNode  # noqa: E402
from restocker_perception.identity.ladder_service import (  # noqa: E402
    STATUS_CODES,
    IdentifyObjectService,
)
from sensor_msgs.msg import Image  # noqa: E402
from ladder_support import (  # noqa: E402,I100
    crop,
    engine,
    make_registry,
)


@pytest.fixture
def registry(tmp_path):
    return make_registry(tmp_path)


def service(registry, lines, **kw):
    return IdentifyObjectService(engine(registry, lines, shortlist_rank=4, **kw))


def test_the_status_codes_match_the_message_constants():
    assert STATUS_CODES == {
        "identified": IdentifyObject.Response.STATUS_IDENTIFIED,
        "not_identified_yet": IdentifyObject.Response.STATUS_NOT_YET,
        "needs_human": IdentifyObject.Response.STATUS_NEEDS_HUMAN,
        "unknown": IdentifyObject.Response.STATUS_UNKNOWN,
    }


# --- over ROS ---------------------------------------------------------------------------------


def image_message(pixels, encoding="rgb8"):
    message = Image()
    message.height, message.width = pixels.shape[:2]
    message.encoding = encoding
    message.step = pixels.shape[1] * 3
    message.data = pixels.tobytes()
    return message


@pytest.fixture(scope="module")
def ros_context():
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=random.Random(os.getpid() + 7).randint(120, 200))
    yield context
    rclpy.shutdown(context=context)


def call(context, node_factory, request):
    node = node_factory(context)
    client_node = rclpy.create_node("identify_object_client", context=context)
    client = client_node.create_client(IdentifyObject, "identify_object")
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(client_node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        assert client.wait_for_service(timeout_sec=10.0)
        future = client.call_async(request)
        for _ in range(400):
            if future.done():
                break
            threading.Event().wait(0.05)
        return future.result()
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()


def test_the_unconfigured_node_answers_unknown(ros_context):
    request = IdentifyObject.Request()
    request.object_key = "o"
    request.crops = [image_message(np.zeros((32, 32, 3), np.uint8))]
    response = call(ros_context, lambda ctx: IdentifyObjectNode(context=ctx), request)
    assert response.status == IdentifyObject.Response.STATUS_UNKNOWN
    assert not response.sidecar_available and not response.has_sku


def test_the_node_returns_guidance_over_messages(ros_context, registry):
    svc = service(registry, {1: ["Fennlo", "Sparkling"]})
    request = IdentifyObject.Request()
    request.object_key = "o"
    request.max_relooks = -1
    request.crops = [image_message(crop(1, 1))]
    request.half_width_px = [0.0]
    request.camera_azimuth_deg = [math.nan]
    response = call(ros_context, lambda ctx: IdentifyObjectNode(service=svc, context=ctx), request)
    assert response.sidecar_available
    assert response.status == IdentifyObject.Response.STATUS_NOT_YET
    assert response.guidance_kind == "turn" and response.has_turn and response.has_facing
    assert response.look_index == 0 and response.registry_sha256 == registry.content_hash()


def test_a_bad_image_is_rejected_not_fatal(ros_context, registry):
    svc = service(registry, {})
    request = IdentifyObject.Request()
    request.object_key = "o"
    request.crops = [image_message(np.zeros((8, 8, 3), np.uint8), "mono8")]
    response = call(ros_context, lambda ctx: IdentifyObjectNode(service=svc, context=ctx), request)
    assert (
        "bad request" in response.reason
        and response.status == IdentifyObject.Response.STATUS_NOT_YET
    )
