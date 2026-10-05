// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <optional>
#include <string>

#include "restocker_perception/fake_viewpoint_port.hpp"

namespace restocker_perception
{
namespace
{

[[nodiscard]] WristCameraMount some_mount()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.11, 0.0, 0.096);
  transform.linear() =
    Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Result<FramedTransform> framed =
    FramedTransform::create(kWristCameraOpticalFrame, kToolFrame, transform);
  EXPECT_TRUE(framed.has_value());
  Result<WristCameraMount> mount = WristCameraMount::create(framed.value());
  EXPECT_TRUE(mount.has_value());
  return mount.value();
}

[[nodiscard]] CameraViewpoint some_viewpoint()
{
  CameraViewpoint viewpoint;
  viewpoint.pose.frame_id = "world";
  viewpoint.pose.pose.translation() = Eigen::Vector3d(0.2, -0.8, 1.4);
  viewpoint.label = "tray_1";
  return viewpoint;
}

TEST(FakeViewpointPort, ResolvesExactlyOnePerSubmission)
{
  FakeViewpointPort port;
  port.set_mount(some_mount());
  EXPECT_TRUE(port.ready());

  std::optional<ViewpointCompletion> completion;
  std::size_t deliveries = 0U;
  const ViewpointSubmitResult submitted = port.submit(
    ViewpointCorrelation{7U}, some_viewpoint(),
    [&completion, &deliveries](ViewpointCompletion result) {
      ++deliveries;
      completion = std::move(result);
    });
  ASSERT_TRUE(static_cast<bool>(submitted));
  // The callback must not run inline from submit(), so a caller's pump never re-enters itself.
  EXPECT_EQ(deliveries, 0U);
  ASSERT_TRUE(port.outstanding());
  EXPECT_EQ(port.submission().viewpoint.label, "tray_1");

  port.complete_with_mount();
  EXPECT_EQ(deliveries, 1U);
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->correlation.request_generation, 7U);
  EXPECT_EQ(completion->outcome, ViewpointOutcome::kResolved);
  EXPECT_EQ(completion->goal.label, "tray_1");
  EXPECT_EQ(completion->goal.pose.frame_id, "world");
  EXPECT_FALSE(port.outstanding());
}

TEST(FakeViewpointPort, RefusesASecondOutstandingSubmission)
{
  FakeViewpointPort port;
  port.set_mount(some_mount());
  ASSERT_TRUE(
    static_cast<bool>(
      port.submit(
        ViewpointCorrelation{1U}, some_viewpoint(), [](ViewpointCompletion) {})));
  const ViewpointSubmitResult second = port.submit(
    ViewpointCorrelation{2U}, some_viewpoint(), [](ViewpointCompletion) {});
  EXPECT_FALSE(static_cast<bool>(second));
  EXPECT_EQ(second.status, ViewpointSubmitStatus::kBusy);
  EXPECT_EQ(port.submit_attempts, 2U);
}

TEST(FakeViewpointPort, ReportsAMissingMountRatherThanGuessingOne)
{
  FakeViewpointPort port;
  EXPECT_FALSE(port.ready());
  std::optional<ViewpointCompletion> completion;
  ASSERT_TRUE(
    static_cast<bool>(
      port.submit(
        ViewpointCorrelation{3U}, some_viewpoint(),
        [&completion](ViewpointCompletion result) {completion = std::move(result);})));
  port.complete_with_mount();
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, ViewpointOutcome::kMountUnavailable);
  EXPECT_STREQ(viewpoint_outcome_name(completion->outcome), "mount unavailable");
}

TEST(FakeViewpointPort, SurfacesAConfiguredRefusalOnceAndOnlyOnce)
{
  FakeViewpointPort port;
  port.set_mount(some_mount());
  port.refuse_next(ViewpointSubmitStatus::kUnavailable, "shutting down");
  const ViewpointSubmitResult refused = port.submit(
    ViewpointCorrelation{4U}, some_viewpoint(), [](ViewpointCompletion) {});
  EXPECT_EQ(refused.status, ViewpointSubmitStatus::kUnavailable);
  EXPECT_EQ(refused.detail, "shutting down");
  EXPECT_TRUE(
    static_cast<bool>(
      port.submit(
        ViewpointCorrelation{5U}, some_viewpoint(), [](ViewpointCompletion) {})));
}

TEST(FakeViewpointPort, DeliversAChosenFailureOutcome)
{
  FakeViewpointPort port;
  port.set_mount(some_mount());
  std::optional<ViewpointCompletion> completion;
  ASSERT_TRUE(
    static_cast<bool>(
      port.submit(
        ViewpointCorrelation{6U}, some_viewpoint(),
        [&completion](ViewpointCompletion result) {completion = std::move(result);})));
  port.cancel();
  port.complete_with_failure(ViewpointOutcome::kCanceled, "asked to stop");
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, ViewpointOutcome::kCanceled);
  EXPECT_EQ(completion->detail, "asked to stop");
  EXPECT_EQ(port.cancel_requests, 1U);
}

}  // namespace
}  // namespace restocker_perception
