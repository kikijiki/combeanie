// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <functional>
#include <string>

#include "restocker_perception/viewpoint_geometry.hpp"

namespace restocker_perception
{

// Correlates one viewpoint resolution with its single completion.
//
// Mirrors the coordinator's OperationCorrelation; this package sits below restocker_task_executor
// and must not depend on it. A caller that owns both carries its own generation into this field.
struct ViewpointCorrelation
{
  std::uint64_t request_generation{0U};
};

enum class ViewpointOutcome : std::uint8_t
{
  // A tool goal was produced from the requested camera pose.
  kResolved,
  // The fixed tool0 <- wrist_camera_optical_frame transform could not be established. Nothing was
  // guessed: a survey that does not know where its camera is bolted must not aim.
  kMountUnavailable,
  // The requested viewpoint is malformed (no frame, a non-finite pose). Never retryable.
  kInvalidRequest,
  kCanceled,
  // The port cannot take work at all, for example during shutdown.
  kUnavailable,
};

[[nodiscard]] const char * viewpoint_outcome_name(ViewpointOutcome outcome) noexcept;

struct ViewpointCompletion
{
  ViewpointCorrelation correlation;
  ViewpointOutcome outcome{ViewpointOutcome::kUnavailable};
  std::string detail;
  // Meaningful only when outcome is kResolved.
  Tool0ViewpointGoal goal;
  // The mount transform this resolution used, reported so a failure's cause is not inferred from
  // the goal it produced.
  Eigen::Isometry3d tool0_from_optical{Eigen::Isometry3d::Identity()};
};

enum class ViewpointSubmitStatus : std::uint8_t
{
  kAccepted,
  kInvalidRequest,
  // A resolution is already outstanding.
  kBusy,
  kUnavailable,
};

struct ViewpointSubmitResult
{
  ViewpointSubmitStatus status{ViewpointSubmitStatus::kUnavailable};
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return status == ViewpointSubmitStatus::kAccepted;
  }
};

// Turns a desired pose of the wrist camera's optical frame into the tool0 pose that achieves it.
//
// This is the boundary at which a viewpoint stops being a camera pose and becomes something a
// motion goal can carry, with the mount transform applied exactly once, in one place, by code that
// knows which side it composes on.
//
// Implementations must invoke the completion callback exactly once per accepted submission, from
// a thread the caller does not own, and must not call back inline from submit().
class ViewpointPort
{
public:
  using CompletionCallback = std::function<void (ViewpointCompletion)>;

  virtual ~ViewpointPort() = default;

  // False until the mount transform has been established. Submission is still allowed; it will
  // complete kMountUnavailable rather than block.
  [[nodiscard]] virtual bool ready() const = 0;

  [[nodiscard]] virtual ViewpointSubmitResult submit(
    ViewpointCorrelation correlation, CameraViewpoint viewpoint, CompletionCallback callback) = 0;

  // Request that an outstanding resolution stop. The completion still arrives.
  virtual void cancel() noexcept = 0;
};

}  // namespace restocker_perception
