// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/tray_survey_logic.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] Eigen::Vector3d position_of(const TrayObservation & observation)
{
  return {
    observation.pose.pose.position.x, observation.pose.pose.position.y,
    observation.pose.pose.position.z};
}

[[nodiscard]] bool same_identity_hypothesis(
  const TrayObservation & left, const TrayObservation & right)
{
  return left.product_class == right.product_class;
}

[[nodiscard]] bool matches_request(
  const TrayObservation & candidate, const std::uint8_t product_class, const bool has_sku,
  const std::string & sku)
{
  if (candidate.status != TrayObservation::STATUS_OK) {
    return false;
  }
  if (product_class != TrayObservation::PRODUCT_CLASS_UNKNOWN &&
    candidate.product_class != product_class)
  {
    return false;
  }
  return !has_sku || (candidate.has_sku && candidate.sku == sku);
}

// Within `radius_m` of any marked point. A non-positive radius disables the marks.
[[nodiscard]] bool excluded_by_mark(
  const TrayObservation & candidate, const std::vector<geometry_msgs::msg::Point> & marks,
  const double radius_m)
{
  if (radius_m <= 0.0 || marks.empty()) {
    return false;
  }
  const Eigen::Vector3d position = position_of(candidate);
  const double radius_squared = radius_m * radius_m;
  return std::ranges::any_of(
    marks, [&position, radius_squared](const geometry_msgs::msg::Point & marked) {
      const Eigen::Vector3d point(marked.x, marked.y, marked.z);
      return (position - point).squaredNorm() <= radius_squared;
    });
}

// Highest confidence first; position breaks ties so the choice never depends on the order the
// stations happened to report in.
[[nodiscard]] bool ranks_before(const TrayObservation & candidate, const TrayObservation & best)
{
  if (candidate.confidence > best.confidence) {
    return true;
  }
  if (candidate.confidence < best.confidence) {
    return false;
  }
  const Eigen::Vector3d position = position_of(candidate);
  const Eigen::Vector3d best_position = position_of(best);
  for (const Eigen::Index axis : {0, 1, 2}) {
    if (position[axis] < best_position[axis]) {
      return true;
    }
    if (position[axis] > best_position[axis]) {
      return false;
    }
  }
  return false;
}

}  // namespace

std::optional<std::int64_t> observation_stamp_ns(const TrayObservation & observation)
{
  const std::int64_t seconds = observation.header.stamp.sec;
  const std::int64_t nanoseconds = observation.header.stamp.nanosec;
  if (seconds == 0 && nanoseconds == 0) {
    return std::nullopt;
  }
  if (seconds < 0 || nanoseconds < 0 || nanoseconds >= 1'000'000'000LL) {
    return std::nullopt;
  }
  return (seconds * 1'000'000'000LL) + nanoseconds;
}

std::vector<TrayObservation> observations_in_window(
  const std::vector<TrayObservation> & raw, const std::string & backend_name,
  const std::int64_t start_ns, const std::int64_t end_ns)
{
  std::vector<TrayObservation> kept;
  kept.reserve(raw.size());
  for (const TrayObservation & observation : raw) {
    if (observation.status != TrayObservation::STATUS_OK) {
      continue;
    }
    if (observation.backend_name != backend_name) {
      continue;
    }
    const auto stamp = observation_stamp_ns(observation);
    if (!stamp) {
      continue;
    }
    if (start_ns > 0 && *stamp <= start_ns) {
      continue;
    }
    if (end_ns > 0 && *stamp > end_ns) {
      continue;
    }
    kept.push_back(observation);
  }
  return kept;
}

std::vector<TrayObservation> merge_candidates(
  const std::vector<TrayObservation> & existing, const std::vector<TrayObservation> & incoming,
  const double merge_radius_m)
{
  std::vector<TrayObservation> merged = existing;
  const double radius = std::max(merge_radius_m, 0.0);
  const double radius_squared = radius * radius;
  for (const TrayObservation & candidate : incoming) {
    std::size_t nearest_index = merged.size();
    double nearest_squared = std::numeric_limits<double>::max();
    for (std::size_t index = 0U; index < merged.size(); ++index) {
      if (!same_identity_hypothesis(merged[index], candidate)) {
        continue;
      }
      const double distance_squared =
        (position_of(merged[index]) - position_of(candidate)).squaredNorm();
      if (distance_squared <= radius_squared && distance_squared < nearest_squared) {
        nearest_index = index;
        nearest_squared = distance_squared;
      }
    }
    if (nearest_index < merged.size()) {
      // The later acquisition of the same product is the estimate that keeps.
      merged[nearest_index] = candidate;
    } else {
      merged.push_back(candidate);
    }
  }
  return merged;
}

std::optional<TrayObservation> select_candidate(
  const std::vector<TrayObservation> & candidates, const std::uint8_t product_class,
  const bool has_sku, const std::string & sku,
  const std::vector<geometry_msgs::msg::Point> & refuted_positions, const double refuted_radius_m,
  const std::vector<geometry_msgs::msg::Point> & skip_mark_positions)
{
  const TrayObservation * best = nullptr;
  for (const TrayObservation & candidate : candidates) {
    if (!matches_request(candidate, product_class, has_sku, sku) ||
      excluded_by_mark(candidate, refuted_positions, refuted_radius_m) ||
      excluded_by_mark(candidate, skip_mark_positions, refuted_radius_m))
    {
      continue;
    }
    if (best == nullptr || ranks_before(candidate, *best)) {
      best = &candidate;
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return *best;
}

namespace
{

// The first STATUS_OK candidate standing ahead of `candidate` in its feed column, or nullptr.
[[nodiscard]] const TrayObservation * blocker_ahead_in_feed_column(
  const TrayObservation & candidate, const std::vector<TrayObservation> & candidates,
  const FeedOrder & feed)
{
  const Eigen::Vector3d in_shelf = feed.shelf_from_planning * position_of(candidate);
  const double same_product_squared =
    std::max(feed.same_product_radius_m, 0.0) * std::max(feed.same_product_radius_m, 0.0);
  for (const TrayObservation & other : candidates) {
    if (&other == &candidate || other.status != TrayObservation::STATUS_OK) {
      continue;
    }
    // A second hypothesis of the same product (the merge radius is below the closest two
    // catalogued products can stand) is not a product in front of it.
    if ((position_of(other) - position_of(candidate)).squaredNorm() <= same_product_squared) {
      continue;
    }
    const Eigen::Vector3d other_in_shelf = feed.shelf_from_planning * position_of(other);
    if (std::abs(other_in_shelf.x() - in_shelf.x()) <= feed.column_half_width_m &&
      other_in_shelf.y() > in_shelf.y() + kFeedAheadToleranceM)
    {
      return &other;
    }
  }
  return nullptr;
}

[[nodiscard]] const char * product_class_label(const std::uint8_t product_class) noexcept
{
  switch (product_class) {
    case TrayObservation::PRODUCT_CLASS_CAN:
      return "can";
    case TrayObservation::PRODUCT_CLASS_SMALL_BOTTLE:
      return "small bottle";
    case TrayObservation::PRODUCT_CLASS_LARGE_BOTTLE:
      return "large bottle";
    default:
      return "product";
  }
}

[[nodiscard]] std::string describe_at(const TrayObservation & observation)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << product_class_label(observation.product_class) << " at ("
         << observation.pose.pose.position.x << ", " << observation.pose.pose.position.y << ")";
  return stream.str();
}

}  // namespace

bool stands_behind_in_feed_column(
  const TrayObservation & candidate, const std::vector<TrayObservation> & candidates,
  const FeedOrder & feed)
{
  return blocker_ahead_in_feed_column(candidate, candidates, feed) != nullptr;
}

std::vector<bool> feed_column_fronts(
  const std::vector<TrayObservation> & candidates, const FeedOrder & feed)
{
  std::vector<bool> fronts;
  fronts.reserve(candidates.size());
  for (const TrayObservation & candidate : candidates) {
    fronts.push_back(
      candidate.status == TrayObservation::STATUS_OK &&
      !stands_behind_in_feed_column(candidate, candidates, feed));
  }
  return fronts;
}

CandidateSelection select_feed_front_candidate(
  const std::vector<TrayObservation> & candidates, const std::uint8_t product_class,
  const bool has_sku, const std::string & sku,
  const std::vector<geometry_msgs::msg::Point> & excluded_positions,
  const double excluded_radius_m, const FeedOrder & feed,
  const std::vector<geometry_msgs::msg::Point> & skip_mark_positions)
{
  CandidateSelection selection;
  const TrayObservation * best = nullptr;
  for (const TrayObservation & candidate : candidates) {
    if (!matches_request(candidate, product_class, has_sku, sku)) {
      continue;
    }
    // Either kind of mark keeps a candidate from being nominated (Card 058 keeps them in
    // separate goal fields only so the detail can name them).
    const bool marked =
      excluded_by_mark(candidate, excluded_positions, excluded_radius_m) ||
      excluded_by_mark(candidate, skip_mark_positions, excluded_radius_m);
    // Every candidate of the goal is a potential blocker, the marked ones included: a mark
    // keeps a product from being nominated, it does not take it out of its column.
    const TrayObservation * const blocker =
      blocker_ahead_in_feed_column(candidate, candidates, feed);
    if (blocker != nullptr && selection.feed_block_example.empty()) {
      selection.feed_block_example =
        describe_at(candidate) + " stands behind " + describe_at(*blocker);
    }
    if (marked) {
      // Marked stock behind a front is still stock behind a front (review cmbrev066b N1).
      if (blocker != nullptr) {
        ++selection.marked_feed_blocked_candidates;
      }
      continue;
    }
    if (blocker != nullptr) {
      ++selection.feed_blocked_candidates;
      continue;
    }
    if (best == nullptr || ranks_before(candidate, *best)) {
      best = &candidate;
    }
  }
  if (best != nullptr) {
    selection.selected = *best;
  }
  return selection;
}

NoCandidateCause classify_no_candidate(
  const std::vector<TrayObservation> & candidates, const std::uint8_t product_class,
  const bool has_sku, const std::string & sku,
  const std::vector<geometry_msgs::msg::Point> & refuted_positions,
  const std::vector<geometry_msgs::msg::Point> & skip_mark_positions, const double radius_m)
{
  bool saw_refuted = false;
  bool saw_skip = false;
  for (const TrayObservation & candidate : candidates) {
    if (!matches_request(candidate, product_class, has_sku, sku)) {
      continue;
    }
    if (excluded_by_mark(candidate, refuted_positions, radius_m)) {
      saw_refuted = true;
    }
    if (excluded_by_mark(candidate, skip_mark_positions, radius_m)) {
      saw_skip = true;
    }
  }
  if (!saw_refuted && !saw_skip) {
    return NoCandidateCause::kNoMatch;
  }
  if (saw_refuted && saw_skip) {
    return NoCandidateCause::kMixed;
  }
  return saw_skip ? NoCandidateCause::kSkipMarksOnly : NoCandidateCause::kRefutationsOnly;
}

std::string no_candidate_detail(const NoCandidateCause cause)
{
  switch (cause) {
    case NoCandidateCause::kNoMatch:
      return "no overview candidate matched the requested selection";
    case NoCandidateCause::kSkipMarksOnly:
      return
        "every overview candidate matching the request was excluded by an active skip mark";
    case NoCandidateCause::kRefutationsOnly:
      return "every overview candidate matching the request was refuted earlier this cycle";
    case NoCandidateCause::kMixed:
      return
        "every overview candidate matching the request was excluded by earlier refutations "
        "and active skip marks";
  }
  return "no overview candidate matched the requested selection";
}

std::string no_candidate_detail(const NoCandidateCause cause, const std::uint32_t feed_blocked)
{
  if (feed_blocked == 0U) {
    return no_candidate_detail(cause);
  }
  // Milestone 10 §6, Card 066: stock stands behind a front this survey could not nominate, which
  // is not "no candidate matched". The marks, when any reached a candidate, are named after it.
  std::string detail =
    "every remaining matching candidate stands behind another tray product in its feed column (" +
    std::to_string(feed_blocked) + " feed-blocked)";
  switch (cause) {
    case NoCandidateCause::kNoMatch:
      break;
    case NoCandidateCause::kSkipMarksOnly:
      detail += "; the others are excluded by an active skip mark";
      break;
    case NoCandidateCause::kRefutationsOnly:
      detail += "; the others were refuted earlier this cycle";
      break;
    case NoCandidateCause::kMixed:
      detail += "; the others are excluded by earlier refutations and active skip marks";
      break;
  }
  return detail;
}

const char * confirmation_verdict_name(const ConfirmationVerdict verdict) noexcept
{
  switch (verdict) {
    case ConfirmationVerdict::kConfirmed:
      return "confirmed";
    case ConfirmationVerdict::kRefutedClassMismatch:
      return "refuted: class mismatch";
    case ConfirmationVerdict::kRefutedSkuMismatch:
      return "refuted: SKU mismatch";
    case ConfirmationVerdict::kRefutedAbsent:
      return "refuted: absent";
    case ConfirmationVerdict::kNoObservation:
      return "no confirmation observation";
  }
  return "unknown";
}

ConfirmationEvaluation evaluate_confirmation(
  const TrayObservation & selected, const std::vector<TrayObservation> & observations,
  const TraySurveyLogicConfig & config)
{
  ConfirmationEvaluation evaluation;
  if (observations.empty()) {
    evaluation.verdict = ConfirmationVerdict::kNoObservation;
    evaluation.detail = "no confirm-duty observation arrived after the arm stopped";
    return evaluation;
  }

  const Eigen::Vector3d target = position_of(selected);
  const double radius = std::max(config.association_radius_m, 0.0);
  std::vector<std::pair<double, const TrayObservation *>> associated;
  const TrayObservation * nearest = nullptr;
  double nearest_distance = std::numeric_limits<double>::max();
  for (const TrayObservation & observation : observations) {
    const double distance = (position_of(observation) - target).norm();
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = &observation;
    }
    if (distance <= radius) {
      associated.emplace_back(distance, &observation);
    }
  }
  if (nearest != nullptr) {
    evaluation.nearest_observation = *nearest;
  }

  if (associated.empty()) {
    evaluation.verdict = ConfirmationVerdict::kRefutedAbsent;
    std::ostringstream detail;
    detail << "the close view saw " << observations.size()
           << " observation(s), none within " << radius << " m of the candidate (nearest at "
           << nearest_distance << " m)";
    evaluation.detail = detail.str();
    return evaluation;
  }

  // Contradiction wins over agreement among the associated observations. Two frames may
  // disagree while the arm settles, or a neighbour's blob may land inside the radius: if
  // anything at the candidate's place claims a different identity, the hypothesis is refused,
  // and the report names the contradicting observation rather than the agreeing one.
  const TrayObservation * class_contradiction = nullptr;
  double class_distance = std::numeric_limits<double>::max();
  const TrayObservation * sku_contradiction = nullptr;
  double sku_distance = std::numeric_limits<double>::max();
  for (const auto & [distance, observation] : associated) {
    if (observation->product_class != selected.product_class) {
      if (distance < class_distance) {
        class_distance = distance;
        class_contradiction = observation;
      }
      continue;
    }
    if (selected.has_sku && observation->has_sku && selected.sku != observation->sku &&
      distance < sku_distance)
    {
      sku_distance = distance;
      sku_contradiction = observation;
    }
  }
  if (class_contradiction != nullptr) {
    evaluation.verdict = ConfirmationVerdict::kRefutedClassMismatch;
    evaluation.associated_observation = *class_contradiction;
    evaluation.associated_distance_m = class_distance;
    evaluation.detail = "close-range product class " +
      std::to_string(static_cast<int>(class_contradiction->product_class)) +
      " contradicts the overview class " +
      std::to_string(static_cast<int>(selected.product_class));
    return evaluation;
  }
  if (sku_contradiction != nullptr) {
    evaluation.verdict = ConfirmationVerdict::kRefutedSkuMismatch;
    evaluation.associated_observation = *sku_contradiction;
    evaluation.associated_distance_m = sku_distance;
    evaluation.detail = "close-range SKU '" + sku_contradiction->sku +
      "' contradicts the overview SKU '" + selected.sku + "'";
    return evaluation;
  }

  // Everything associated agrees: the nearest frame is the replacing observation.
  const auto & [distance, confirmation] = *std::min_element(
    associated.begin(), associated.end(),
    [](const auto & left, const auto & right) {return left.first < right.first;});
  evaluation.verdict = ConfirmationVerdict::kConfirmed;
  evaluation.associated_observation = *confirmation;
  evaluation.associated_distance_m = distance;
  evaluation.detail = "close-range identity agrees with the overview hypothesis at " +
    std::to_string(distance) + " m from the estimated candidate position";
  return evaluation;
}

const char * classify_drained_station(
  const restocker_interfaces::msg::TrayStationAcquisition & report) noexcept
{
  // One label per row of TrayStationAcquisition.msg's stage table, read from the counts alone.
  // Arrival timing (late_images/late_published) is printed beside the label but never picks it:
  // an image that arrived during the drain does not make an unpublished frame an admission
  // question. The two "empty view" rows are the pipeline working, not a stage fault: the station
  // is declared empty fail-closed (Card 064: a genuinely empty tray whose only proposals were
  // non-product components the geometry gates refused). A tap that is not configured reads zero
  // without meaning it, so it names no stage.
  if (report.admitted > 0U) {
    return "late delivery: in-window frame(s) arrived after arrival_grace_ms and are admitted";
  }
  if (report.image_tap_configured && report.images == 0U) {
    return "input gap: no colour frame carried a stamp in the dwell window (camera or bridge "
           "published nothing for this station)";
  }
  if (report.published == 0U) {
    if (!report.detection_tap_configured) {
      return "nothing published: the detection tap is not configured, so the stage that "
             "produced no observation cannot be named";
    }
    if (report.detection_frames == 0U) {
      return "duty not processing: in-window images, but the overview duty emitted no detection "
             "frame";
    }
    if (report.frames_with_detections == 0U) {
      return "empty view: the colour stage found nothing in view (no proposal in any in-window "
             "detection frame)";
    }
    return "empty view: estimation refused every proposal (no proposal passed the duty's "
           "geometry gates; its throttled refusal log names the gate)";
  }
  return "admission mismatch: in-window observations were published but none met the admission "
         "rules (backend/status/stamp)";
}

const char * absence_probe_verdict_name(const AbsenceProbeVerdict verdict) noexcept
{
  switch (verdict) {
    case AbsenceProbeVerdict::kNotEvaluated:
      return "not evaluated";
    case AbsenceProbeVerdict::kSeen:
      return "seen";
    case AbsenceProbeVerdict::kViewedEmpty:
      return "viewed empty";
    case AbsenceProbeVerdict::kOccluded:
      return "occluded";
    case AbsenceProbeVerdict::kNotCovered:
      return "not covered";
    case AbsenceProbeVerdict::kNoEvidence:
      return "no evidence";
  }
  return "unknown";
}

bool station_acquisition_healthy(
  const restocker_interfaces::msg::TrayStationAcquisition & report) noexcept
{
  if (!report.image_tap_configured || !report.detection_tap_configured) {
    return false;
  }
  return report.images > 0U && report.detection_frames > 0U &&
         report.frames_with_detections > 0U && report.admitted > 0U;
}

AbsenceProbeVerdict classify_absence_probe(
  const Eigen::Vector3d & probe, const std::vector<Eigen::Vector3d> & candidates,
  const std::vector<Eigen::Vector3d> & occluders,
  const std::vector<AbsenceStationView> & stations, const AbsenceProbeConfig & config)
{
  for (const Eigen::Vector3d & candidate : candidates) {
    if ((candidate - probe).norm() <= config.seen_radius_m) {
      return AbsenceProbeVerdict::kSeen;
    }
  }
  bool framed_healthy = false;
  bool framed_unhealthy = false;
  for (const AbsenceStationView & station : stations) {
    const restocker_perception::FramingResult framing = restocker_perception::frames_region(
      config.frustum, station.world_from_optical, {probe}, config.framing_margin_px);
    if (!framing.framed) {
      continue;
    }
    if (!station.healthy) {
      framed_unhealthy = true;
      continue;
    }
    framed_healthy = true;
    // The line of sight is the segment from the camera origin to the probe. A point blocks it
    // when it lies in front of the probe along the ray and within the occluder radius of it.
    const Eigen::Isometry3d optical_from_world = station.world_from_optical.inverse();
    const Eigen::Vector3d target = optical_from_world * probe;
    const double range = target.norm();
    if (range <= 0.0) {
      continue;
    }
    const Eigen::Vector3d direction = target / range;
    const bool blocked = std::ranges::any_of(
      occluders, [&](const Eigen::Vector3d & point) {
        const Eigen::Vector3d local = optical_from_world * point;
        const double along = local.dot(direction);
        if (along <= 0.0 || along >= range) {
          return false;
        }
        return (local - along * direction).norm() < config.occluder_radius_m;
      });
    if (!blocked) {
      return AbsenceProbeVerdict::kViewedEmpty;
    }
  }
  if (framed_healthy) {
    return AbsenceProbeVerdict::kOccluded;
  }
  return framed_unhealthy ? AbsenceProbeVerdict::kNoEvidence : AbsenceProbeVerdict::kNotCovered;
}

}  // namespace restocker_task_executor
