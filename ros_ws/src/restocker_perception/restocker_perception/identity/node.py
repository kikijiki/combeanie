# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""/identify_product: ROS 2 service node around :class:`IdentifyService`."""
# Parameters: registry_path, calibration_path, whitener_path (all empty = no sidecar, so the node
# answers unknown), backend (siglip2 or fake), backend_revision, device, ocr (rapidocr or none),
# barcode (none or zxing: the decoder for barcode_images; none ignores them) and barcode_enabled
# (default true; false leaves the barcode channel out of every decision).
# The default world-state identity path does not call this yet (CMB-117 is the cutover).

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from restocker_interfaces.srv import IdentifyProduct

from restocker_perception.identity.barcode import ZxingBarcodeBackend
from restocker_perception.identity.service import IdentifyService, image_to_rgb
from restocker_perception.registry.backends import (
    FakeEmbeddingBackend,
    RapidOcrBackend,
    SigLip2Backend,
)


def _convert(images) -> list:
    return [image_to_rgb(i.height, i.width, i.encoding, i.step, bytes(i.data)) for i in images]


class IdentifyProductNode(Node):
    def __init__(self, service: IdentifyService | None = None, **node_options) -> None:
        super().__init__("identify_product", **node_options)
        for name, default in (
            ("registry_path", ""),
            ("calibration_path", ""),
            ("whitener_path", ""),
            ("backend", "siglip2"),
            ("backend_revision", "main"),
            ("device", "cpu"),
            ("ocr", "rapidocr"),
            ("ocr_version", "unpinned"),
            ("barcode", "none"),
            ("barcode_version", "unpinned"),
            ("barcode_enabled", True),
        ):
            self.declare_parameter(name, default)
        self.service = service or self._load_service()
        self.create_service(IdentifyProduct, "identify_product", self._handle)
        self.get_logger().info(
            "identify_product ready"
            if self.service.available
            else f"identify_product answering unknown: {self.service.unavailable_reason}"
        )

    def _load_service(self) -> IdentifyService:
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        if not (get("registry_path") and get("calibration_path") and get("whitener_path")):
            return IdentifyService(None, unavailable_reason="identity sidecar not configured")
        backend = (
            FakeEmbeddingBackend()
            if get("backend") == "fake"
            else SigLip2Backend(revision=get("backend_revision"), device=get("device"))
        )
        ocr = RapidOcrBackend(version=get("ocr_version")) if get("ocr") == "rapidocr" else None
        barcode = (
            ZxingBarcodeBackend(version=get("barcode_version"))
            if get("barcode") == "zxing"
            else None
        )
        return IdentifyService.load(
            get("registry_path"),
            get("calibration_path"),
            get("whitener_path"),
            backend,
            ocr,
            barcode_backend=barcode,
            barcode_enabled=bool(get("barcode_enabled")),
        )

    def _handle(self, request, response):
        try:
            reply = self.service.identify(
                _convert(request.crops), _convert(request.barcode_images)
            )
        except ValueError as error:  # unsupported encoding or malformed image
            reply = self.service.reject(f"bad request: {error}")
        for key, value in reply.items():
            setattr(response, key, value)
        return response


def main(argv=None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = IdentifyProductNode()
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
