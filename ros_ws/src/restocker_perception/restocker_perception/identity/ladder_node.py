# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""/identify_object: ROS 2 service node around :class:`IdentifyObjectService` (the ladder)."""
# Parameters: registry_path, whitener_path (both empty = no sidecar, so the node answers unknown),
# config_path (ladder config JSON; empty = defaults), backend (siglip2 or fake), backend_revision,
# device, ocr_version, ocr_threads, barcode (none or zxing: the decoder for barcode_images) and
# barcode_enabled (default true; false leaves the barcode rung out of every decision).

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from restocker_interfaces.srv import IdentifyObject

from restocker_perception.identity.barcode import ZxingBarcodeBackend
from restocker_perception.identity.ladder_service import IdentifyObjectService
from restocker_perception.identity.service import image_to_rgb
from restocker_perception.registry.backends import (
    FakeEmbeddingBackend,
    RapidOcrBackend,
    SigLip2Backend,
)

# RapidOCR thresholds lowered from its defaults: label text of a crop is a handful of pixels high.
OCR_OVERRIDES = {"text_score": 0.3, "box_thresh": 0.3, "unclip_ratio": 2.0, "thresh": 0.2}


def _convert(images) -> list:
    return [image_to_rgb(i.height, i.width, i.encoding, i.step, bytes(i.data)) for i in images]


class IdentifyObjectNode(Node):
    def __init__(self, service: IdentifyObjectService | None = None, **node_options) -> None:
        super().__init__("identify_object", **node_options)
        for name, default in (
            ("registry_path", ""),
            ("whitener_path", ""),
            ("config_path", ""),
            ("backend", "siglip2"),
            ("backend_revision", "main"),
            ("device", "cpu"),
            ("ocr_version", "unpinned"),
            ("ocr_threads", 3),
            ("barcode", "none"),
            ("barcode_version", "unpinned"),
            ("barcode_enabled", True),
        ):
            self.declare_parameter(name, default)
        self.service = service or self._load_service()
        self.create_service(IdentifyObject, "identify_object", self._handle)
        self.get_logger().info(
            "identify_object ready"
            if self.service.available
            else f"identify_object answering unknown: {self.service.unavailable_reason}"
        )

    def _load_service(self) -> IdentifyObjectService:
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        if not (get("registry_path") and get("whitener_path")):
            return IdentifyObjectService(
                None, unavailable_reason="identity sidecar not configured"
            )
        embedding = (
            FakeEmbeddingBackend()
            if get("backend") == "fake"
            else SigLip2Backend(revision=get("backend_revision"), device=get("device"))
        )
        ocr = RapidOcrBackend(
            get("ocr_version"), overrides=OCR_OVERRIDES, threads=int(get("ocr_threads"))
        )
        barcode = (
            ZxingBarcodeBackend(version=get("barcode_version"))
            if get("barcode") == "zxing"
            else None
        )
        return IdentifyObjectService.load(
            get("registry_path"),
            get("whitener_path"),
            get("config_path"),
            embedding,
            ocr,
            barcode_backend=barcode,
            barcode_enabled=bool(get("barcode_enabled")),
        )

    def _handle(self, request, response):
        try:
            reply = self.service.identify(
                request.object_key,
                _convert(request.crops),
                list(request.half_width_px),
                list(request.camera_azimuth_deg),
                _convert(request.barcode_images),
                reset=bool(request.reset),
                max_relooks=int(request.max_relooks),
            )
        except ValueError as error:  # unsupported encoding or malformed image
            reply = self.service.reject(f"bad request: {error}")
        for key, value in reply.items():
            setattr(response, key, value)
        return response


def main(argv=None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = IdentifyObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
