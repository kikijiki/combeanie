# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Contract tests for the detection, segmentation, and evaluation boundaries."""

from restocker_interfaces.msg import (
    ObjectDetection,
    ObjectObservation,
    ObjectSegmentation,
    PerceptionFrame,
    PoseErrorSample,
)


def test_detection_product_classes_match_the_observation_boundary() -> None:
    # A detection and the observation derived from it must never disagree about category.
    assert ObjectDetection.PRODUCT_CLASS_UNKNOWN == ObjectObservation.PRODUCT_CLASS_UNKNOWN
    assert ObjectDetection.PRODUCT_CLASS_CAN == ObjectObservation.PRODUCT_CLASS_CAN
    assert (
        ObjectDetection.PRODUCT_CLASS_SMALL_BOTTLE == ObjectObservation.PRODUCT_CLASS_SMALL_BOTTLE
    )
    assert (
        ObjectDetection.PRODUCT_CLASS_LARGE_BOTTLE == ObjectObservation.PRODUCT_CLASS_LARGE_BOTTLE
    )


def test_a_detection_carries_no_stamp_or_frame_of_its_own() -> None:
    # Only PerceptionFrame owns when and where.
    detection = ObjectDetection()
    assert not hasattr(detection, "header")
    assert not hasattr(detection, "stamp")

    mask = ObjectSegmentation()
    assert not hasattr(mask, "header")
    assert not hasattr(mask, "frame_id")


def test_perception_frame_owns_the_stamp_and_frame_for_every_detection() -> None:
    frame = PerceptionFrame()
    assert frame.header.frame_id == ""
    assert frame.detections == []
    assert frame.backend_name == ""
    assert frame.backend_version == ""
    assert frame.status == PerceptionFrame.STATUS_OK


def test_perception_frame_status_values_are_unique_and_ok_is_zero() -> None:
    values = {
        PerceptionFrame.STATUS_OK,
        PerceptionFrame.STATUS_INVALID_FRAME,
        PerceptionFrame.STATUS_BACKEND_UNAVAILABLE,
        PerceptionFrame.STATUS_MISSING_TIMESTAMP,
        PerceptionFrame.STATUS_INTERNAL_ERROR,
    }
    assert len(values) == 5
    assert PerceptionFrame.STATUS_OK == 0


def test_a_default_detection_is_not_implicitly_valid() -> None:
    detection = ObjectDetection()
    # An empty bounding box, so a default-constructed detection cannot pass validation.
    assert detection.x_min == detection.x_max == 0
    assert detection.y_min == detection.y_max == 0
    assert detection.product_class == ObjectDetection.PRODUCT_CLASS_UNKNOWN
    assert detection.score == 0.0
    assert detection.instance_id == 0
    assert detection.source_object_id == ""
    assert detection.has_mask is False


def test_pose_error_sample_records_both_stamps_it_compared() -> None:
    # Pose error is only interpretable if the pairing it came from is visible.
    sample = PoseErrorSample()
    assert sample.status == PoseErrorSample.STATUS_OK
    assert sample.source_object_id == ""
    assert sample.translation_error_m == 0.0
    assert sample.estimate_stamp.sec == 0
    assert sample.ground_truth_stamp.sec == 0
    # A default-constructed sample has measured nothing, and its orientation status must say so.
    # The float64 wire default 0.0 reads as a perfect angular error, so the evaluator overwrites
    # axis_error_rad with NaN on every path; this asserts the paired enumerator means "not
    # measured".
    assert sample.orientation_status == PoseErrorSample.ORIENTATION_NOT_ESTIMATED
    # observation_range_m lets the evaluator break reports out by range. The field default is 0.0;
    # the evaluator writes NaN when range is unknown.
    assert hasattr(sample, "observation_range_m")
    assert sample.observation_range_m == 0.0


def test_pose_error_orientation_status_distinguishes_absent_from_zero() -> None:
    # The two must differ: an estimator publishing a constant orientation and one estimating it
    # perfectly both report 0.0 rad, and only this field separates them.
    assert PoseErrorSample.ORIENTATION_NOT_ESTIMATED != PoseErrorSample.ORIENTATION_AXIS_ESTIMATED
    assert PoseErrorSample.ORIENTATION_NOT_ESTIMATED == 0


def test_pose_error_status_values_are_unique_and_ok_is_zero() -> None:
    values = {
        PoseErrorSample.STATUS_OK,
        PoseErrorSample.STATUS_NO_GROUND_TRUTH,
        PoseErrorSample.STATUS_FRAME_MISMATCH,
        PoseErrorSample.STATUS_STALE_PAIRING,
        PoseErrorSample.STATUS_INTERNAL_ERROR,
    }
    assert len(values) == 5
    assert PoseErrorSample.STATUS_OK == 0
