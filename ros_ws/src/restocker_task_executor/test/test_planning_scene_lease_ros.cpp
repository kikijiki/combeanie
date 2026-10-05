// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>

#include <restocker_interfaces/msg/planning_scene_lease.hpp>
#include <restocker_interfaces/msg/planning_scene_lease_operation_status.hpp>

#include "restocker_task_executor/planning_scene_lease_ros.hpp"

namespace restocker_task_executor
{
namespace
{

TEST(PlanningSceneLeaseRos, ConvertsEveryStableResultCode)
{
  using Code = PlanningSceneLeaseCode;
  using Ros = restocker_interfaces::msg::PlanningSceneLeaseOperationStatus;
  const std::array cases{
    std::pair{Code::Unset, Ros::UNSET},
    std::pair{Code::Draining, Ros::DRAINING},
    std::pair{Code::Granted, Ros::GRANTED},
    std::pair{Code::Valid, Ros::VALID},
    std::pair{Code::ReleaseAccepted, Ros::RELEASE_ACCEPTED},
    std::pair{Code::InvalidArgument, Ros::INVALID_ARGUMENT},
    std::pair{Code::Conflict, Ros::CONFLICT},
    std::pair{Code::TokenMismatch, Ros::TOKEN_MISMATCH},
    std::pair{Code::IdempotencyConflict, Ros::IDEMPOTENCY_CONFLICT},
    std::pair{Code::ResourceExhausted, Ros::RESOURCE_EXHAUSTED},
    std::pair{Code::InternalError, Ros::INTERNAL_ERROR},
  };
  for (const auto & [code, expected] : cases) {
    const auto message = to_ros_message(code, "typed detail");
    EXPECT_EQ(message.code, expected);
    EXPECT_EQ(message.detail, "typed detail");
  }
}

TEST(PlanningSceneLeaseRos, ConvertsTokenFreeSummaryAndTimestamp)
{
  const PlanningSceneLeaseSummary summary{
    7, "acquire-7", 11, 13, 5, 2'000'000'003, PlanningSceneLeasePhase::Held};
  const auto message = to_ros_message(summary);
  EXPECT_EQ(message.lease_id, 7U);
  EXPECT_EQ(message.acquisition_operation_id, "acquire-7");
  EXPECT_EQ(message.minimum_applied_revision, 11U);
  EXPECT_EQ(message.granted_applied_revision, 13U);
  EXPECT_EQ(message.verification_epoch, 5U);
  EXPECT_EQ(message.acquired_at.sec, 2);
  EXPECT_EQ(message.acquired_at.nanosec, 3U);
  EXPECT_EQ(message.phase, message.PHASE_HELD);
}

TEST(PlanningSceneLeaseRos, TranslatesRequestsWithoutDroppingRevisionContext)
{
  restocker_interfaces::srv::AcquirePlanningSceneLease::Request acquire_message;
  acquire_message.operation_id = "acquire";
  acquire_message.minimum_applied_revision = 21;
  EXPECT_EQ(
    from_ros_request(acquire_message),
    (AcquirePlanningSceneLeaseRequest{"acquire", 21}));

  restocker_interfaces::srv::ReleasePlanningSceneLease::Request release_message;
  release_message.operation_id = "release";
  release_message.token = "capability";
  release_message.required_semantic_revision = 34;
  EXPECT_EQ(
    from_ros_request(release_message),
    (ReleasePlanningSceneLeaseRequest{"release", "capability", 34}));
}

TEST(PlanningSceneLeaseRos, ReturnsCapabilityOnlyForGrantedAcquisition)
{
  const PlanningSceneLeaseSummary summary{
    1, "acquire", 2, 3, 4, 5, PlanningSceneLeasePhase::Held};
  restocker_interfaces::srv::AcquirePlanningSceneLease::Response response;
  populate_ros_response(
    PlanningSceneLeaseReply{
        PlanningSceneLeaseCode::Draining, "pending", summary, "must-not-leak"},
    response);
  EXPECT_EQ(response.status.code, response.status.DRAINING);
  EXPECT_TRUE(response.has_lease);
  EXPECT_TRUE(response.token.empty());

  populate_ros_response(
    PlanningSceneLeaseReply{
        PlanningSceneLeaseCode::Granted, "held", summary,
        "11111111111111111111111111111111"},
    response);
  EXPECT_EQ(response.status.code, response.status.GRANTED);
  EXPECT_EQ(response.token, "11111111111111111111111111111111");

  restocker_interfaces::srv::ValidatePlanningSceneLease::Response validation;
  populate_ros_response(
    PlanningSceneLeaseReply{
        PlanningSceneLeaseCode::Valid, "valid", summary,
        "11111111111111111111111111111111"},
    validation);
  EXPECT_EQ(validation.status.code, validation.status.VALID);
  EXPECT_TRUE(validation.has_lease);

  restocker_interfaces::srv::ReleasePlanningSceneLease::Response release;
  populate_ros_response(
    PlanningSceneLeaseReply{
        PlanningSceneLeaseCode::ReleaseAccepted, "releasing", summary,
        "11111111111111111111111111111111"},
    release);
  EXPECT_EQ(release.status.code, release.status.RELEASE_ACCEPTED);
  EXPECT_TRUE(release.has_lease);
}

}  // namespace
}  // namespace restocker_task_executor
