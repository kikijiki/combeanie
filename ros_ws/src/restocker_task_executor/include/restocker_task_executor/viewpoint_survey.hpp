// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <restocker_perception/viewpoint_geometry.hpp>
#include <restocker_perception/viewpoint_port.hpp>

#include "restocker_task_executor/motion_port.hpp"

namespace restocker_task_executor
{

struct ViewpointSurveyConfig
{
  // Goal tolerances, set by the framing margin of the tightest survey station. A pose error costs
  // pixels. The tightest station is a tray overview: 46.8 px of margin at a 0.418 m near range,
  // with a focal length of 700.9 px. That converts the pixel budget into metres and radians:
  //
  //   a position error e costs  focal * e / range  = 1.68 px per mm
  //   an orientation error t costs  focal * t      = 0.70 px per mrad, at any range
  //   and t also moves the lens itself, by the 0.146 m mount offset: 0.25 px per mrad more
  //
  // MoveIt's orientation tolerance is per axis, so the geodesic error it permits is up to
  // sqrt(3) times it. At 0.006 m and 0.010 rad the worst case a successful plan may leave is
  // 10.1 + 12.1 + 4.2 = 26.4 px, which fits inside 46.8 with room for controller tracking error.
  double position_tolerance_m{0.006};
  double orientation_tolerance_rad{0.010};
  double velocity_scaling{0.2};
  double acceleration_scaling{0.2};
  std::chrono::milliseconds planning_time{std::chrono::seconds(5)};
};

enum class SurveyOutcome : std::uint8_t
{
  // The segment ran and the controllers reported success. Camera placement is confirmed by
  // measuring TF, not by this outcome.
  kArrived,
  // The camera pose could not be turned into a tool goal. Nothing was commanded.
  kViewpointUnresolved,
  kPlanningFailed,
  kExecutionFailed,
  kCanceled,
  kTimedOut,
  kUnavailable,
  kInvalidRequest,
};

[[nodiscard]] const char * survey_outcome_name(SurveyOutcome outcome) noexcept;

// True when the outcome proves no joint command was issued for this survey.
[[nodiscard]] bool survey_definitely_not_started(SurveyOutcome outcome) noexcept;

struct SurveyCompletion
{
  OperationCorrelation correlation;
  SurveyOutcome outcome{SurveyOutcome::kUnavailable};
  std::string detail;
  // The requested camera pose, echoed so a failure report names the viewpoint, not the derived
  // tool pose.
  restocker_perception::CameraViewpoint commanded_viewpoint;
  // Meaningful only when viewpoint_resolved is true.
  bool viewpoint_resolved{false};
  restocker_perception::Tool0ViewpointGoal commanded_goal;
  bool execution_reached_terminal_stop{false};
};

enum class SurveySubmitStatus : std::uint8_t
{
  kAccepted,
  kInvalidRequest,
  kBusy,
  kUnavailable,
};

struct SurveySubmitResult
{
  SurveySubmitStatus status{SurveySubmitStatus::kUnavailable};
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return status == SurveySubmitStatus::kAccepted;
  }
};

// One survey station visit: resolve a camera pose to a tool goal, then plan and execute to it.
//
// It composes two ports and adds no authority of its own. The segment is an ordinary kFreeSpace
// motion, so it passes the same planning-scene authority gate as every other segment: a survey
// cannot execute against geometry the projector will not certify.
//
// Exactly one completion per accepted submission, delivered from a port's worker thread. Never
// inline from submit(), and never while holding the caller's pump.
class ViewpointSurvey
{
public:
  using CompletionCallback = std::function<void (SurveyCompletion)>;

  ViewpointSurvey(
    restocker_perception::ViewpointPort & viewpoints, MotionPort & motion,
    ViewpointSurveyConfig config = {});

  ViewpointSurvey(const ViewpointSurvey &) = delete;
  ViewpointSurvey & operator=(const ViewpointSurvey &) = delete;
  ViewpointSurvey(ViewpointSurvey &&) = delete;
  ViewpointSurvey & operator=(ViewpointSurvey &&) = delete;
  ~ViewpointSurvey() = default;

  // False while either port is unreachable. Submission is still allowed and completes with a
  // terminal outcome rather than blocking.
  [[nodiscard]] bool ready() const;

  // `overrides` replaces the configured tolerances for this survey only; a non-positive field
  // keeps the configured default.
  [[nodiscard]] SurveySubmitResult submit(
    OperationCorrelation correlation, restocker_perception::CameraViewpoint viewpoint,
    ViewpointSurveyConfig overrides, CompletionCallback callback);

  [[nodiscard]] SurveySubmitResult submit(
    OperationCorrelation correlation, restocker_perception::CameraViewpoint viewpoint,
    CompletionCallback callback)
  {
    return submit(correlation, std::move(viewpoint), ViewpointSurveyConfig{}, std::move(callback));
  }

  // Request that an outstanding survey stop. The completion still arrives.
  void cancel() noexcept;

private:
  struct Active
  {
    OperationCorrelation correlation;
    restocker_perception::CameraViewpoint viewpoint;
    ViewpointSurveyConfig config;
    CompletionCallback callback;
  };

  void on_viewpoint_resolved(restocker_perception::ViewpointCompletion completion);
  void on_motion_complete(MotionCompletion completion);
  // Takes the active survey and delivers its single completion. Called with the lock released.
  void finish(
    SurveyOutcome outcome, std::string detail,
    const std::optional<restocker_perception::Tool0ViewpointGoal> & goal,
    bool execution_reached_terminal_stop);

  restocker_perception::ViewpointPort & viewpoints_;
  MotionPort & motion_;
  ViewpointSurveyConfig config_;

  mutable std::mutex mutex_;
  std::optional<Active> active_;
  std::optional<restocker_perception::Tool0ViewpointGoal> resolved_goal_;
  bool cancel_requested_{false};
};

}  // namespace restocker_task_executor
