// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include <restocker_interfaces/msg/object_observation.hpp>
#include <restocker_interfaces/msg/tray_station_acquisition.hpp>
#include <restocker_perception/viewpoint_geometry.hpp>

namespace restocker_task_executor
{

// The stream and selection rules of one coordinated tray survey, as pure functions over the
// messages that stream carries. The node owns the motion legs and the clocks; everything that
// decides what a candidate is, which one was selected, and whether the close view confirmed or
// refuted it lives here, where it can be tested without a ROS graph, a camera, or an arm.
//
// A candidate is an ObjectObservation that has not been, and must never become, world state:
// these functions key nothing on source_object_id and mint no identity.

using TrayObservation = restocker_interfaces::msg::ObjectObservation;

struct TraySurveyLogicConfig
{
  // The backend each duty stamps its observations with. The survey refuses a stream entry whose
  // backend does not match, so simulator ground truth or the overhead pipeline sharing a topic
  // cannot masquerade as a tray duty.
  std::string overview_backend_name{"wrist_rgbd_tray_overview"};
  std::string confirmation_backend_name{"wrist_rgbd_tray_confirm"};
  // Two acquisitions of one product from a station, or the same product seen through both
  // overview stations' overlap, merge when their estimates land this close together. Below half
  // the smallest centre-to-centre distance two distinct catalogued products can stand at (the
  // can's 0.088 m at the shipped 0.02 m surface separation), so distinct products never merge.
  double candidate_merge_radius_m{0.04};
  // How close the close view must place a product to the selected candidate's estimated position
  // for that observation to be *the* candidate rather than a neighbour. Well below the shipped
  // product spacing, well above the estimator's millimetre-scale noise at confirm range.
  double association_radius_m{0.10};
  // Milestone 10 §6, Card 066: the half width of a tray feed column in the shelf frame (see
  // FeedOrder below).
  double feed_column_half_width_m{0.0225};
};

// The acquisition stamp of an observation, in nanoseconds since epoch, or nullopt when the
// message carries no usable stamp. Windows are half-open: (start_ns, end_ns].
[[nodiscard]] std::optional<std::int64_t> observation_stamp_ns(const TrayObservation & observation);

// Keep status-OK observations from `backend_name` whose stamp falls in (start_ns, end_ns], in
// arrival order. start_ns <= 0 disables the lower bound; end_ns <= 0 disables the upper one.
[[nodiscard]] std::vector<TrayObservation> observations_in_window(
  const std::vector<TrayObservation> & raw, const std::string & backend_name,
  std::int64_t start_ns, std::int64_t end_ns);

// Fold `incoming` into `existing`: an incoming observation whose position lands within
// `merge_radius_m` of an existing candidate of the same product class replaces it (a later
// acquisition of the same product is the better estimate), anything else appends. First-seen
// order is preserved, so the result is deterministic for a deterministic input order.
[[nodiscard]] std::vector<TrayObservation> merge_candidates(
  const std::vector<TrayObservation> & existing, const std::vector<TrayObservation> & incoming,
  double merge_radius_m);

// The one candidate to confirm: highest confidence first, then smallest planning-frame X, Y, Z,
// so a tie is broken by position rather than by arrival order. A requested class of 0 accepts
// any class. A requested SKU applies only to candidates that report one; a candidate that cannot
// show the requested SKU does not match it.
//
// `refuted_positions` marks candidates refused by an earlier confirmation this cycle:
// a candidate within `refuted_radius_m` of any marked point of either list is skipped, so the
// next selection is a reselection rather than the same doomed proposal.
// `skip_mark_positions` marks campaign product skips ("try others first"): excluded exactly like
// refutations, but liftable by the skip-recovery rung (Milestone 10 §6, Card 058), which is why
// the lists stay separate. A non-positive radius disables both lists; empty lists accept every
// candidate.
[[nodiscard]] std::optional<TrayObservation> select_candidate(
  const std::vector<TrayObservation> & candidates, std::uint8_t product_class, bool has_sku,
  const std::string & sku,
  const std::vector<geometry_msgs::msg::Point> & refuted_positions = {},
  double refuted_radius_m = 0.0,
  const std::vector<geometry_msgs::msg::Point> & skip_mark_positions = {});

// The honest OUTCOME_NO_CANDIDATE detail, pinned per exclusion kind (Milestone 10 §6,
// Card 058): which exclusions actually reach the class-matching candidates select_candidate
// refused. Called only when select_candidate returned nullopt, so every class-matching
// candidate it reports is excluded by at least one list.
enum class NoCandidateCause : std::uint8_t
{
  // No class-matching candidate at all (empty overview, or nothing matched class/SKU).
  kNoMatch,
  // Every class-matching candidate sits inside a skip mark and none inside a refutation.
  kSkipMarksOnly,
  // Every class-matching candidate sits inside a refutation and none inside a skip mark.
  kRefutationsOnly,
  // Both kinds of exclusion reach the class-matching candidates.
  kMixed,
};

[[nodiscard]] NoCandidateCause classify_no_candidate(
  const std::vector<TrayObservation> & candidates, std::uint8_t product_class, bool has_sku,
  const std::string & sku,
  const std::vector<geometry_msgs::msg::Point> & refuted_positions,
  const std::vector<geometry_msgs::msg::Point> & skip_mark_positions, double radius_m);

[[nodiscard]] std::string no_candidate_detail(NoCandidateCause cause);

// The same detail when `feed_blocked` matching candidates stood behind a feed-column front
// (Milestone 10 §6, Card 066): feed order is named first, whatever the marks say, because
// "no candidate matched" would be untrue. Zero gives the plain detail above.
[[nodiscard]] std::string no_candidate_detail(NoCandidateCause cause, std::uint32_t feed_blocked);

// Milestone 10 §6, Card 066: a tray column is a feed queue. The coordinator refuses a product
// while another tray product stands in front of it (larger shelf y, same shelf-frame column), and
// Card 037's pin leaves it no other product to take, so the survey must only nominate fronts.
struct FeedOrder
{
  // Maps planning-frame candidate positions into the shelf frame the column rule is judged in.
  Eigen::Isometry3d shelf_from_planning{Eigen::Isometry3d::Identity()};
  // Two candidates share a column when their shelf-frame x differ by at most this. Half the
  // largest catalogued radius, so never narrower than the coordinator's 0.5·r.
  double column_half_width_m{0.0225};
  // A candidate this close is the same product under another hypothesis, not one in front of it.
  double same_product_radius_m{0.04};
};

// The coordinator's own "ahead" slack (task_selection.cpp): larger shelf y by more than this.
inline constexpr double kFeedAheadToleranceM = 0.001;

// Whether some other STATUS_OK candidate in `candidates` stands ahead of `candidate` in its feed
// column. Class, refutation and skip marks do not matter: anything standing there blocks.
[[nodiscard]] bool stands_behind_in_feed_column(
  const TrayObservation & candidate, const std::vector<TrayObservation> & candidates,
  const FeedOrder & feed);

struct CandidateSelection
{
  std::optional<TrayObservation> selected;
  // Class/SKU-matching candidates, not themselves excluded by a position mark, that were passed
  // over only because another candidate stands ahead of them in their feed column.
  std::uint32_t feed_blocked_candidates{0U};
  // Class/SKU-matching candidates under a refutation or skip mark that also stand behind another
  // candidate in their feed column (review cmbrev066b N1).
  std::uint32_t marked_feed_blocked_candidates{0U};
  // The first blocked matching candidate, marked or not, and the candidate ahead of it: class
  // and planning-frame position of each. Empty when nothing matching was blocked.
  std::string feed_block_example;
};

// `select_candidate` restricted to feed-column fronts. A candidate within `excluded_radius_m`
// of a refutation or of a skip mark is never nominated, but it still blocks the column behind
// it; `feed_blocked_candidates` counts only candidates under neither kind of mark.
[[nodiscard]] CandidateSelection select_feed_front_candidate(
  const std::vector<TrayObservation> & candidates, std::uint8_t product_class, bool has_sku,
  const std::string & sku, const std::vector<geometry_msgs::msg::Point> & excluded_positions,
  double excluded_radius_m, const FeedOrder & feed,
  const std::vector<geometry_msgs::msg::Point> & skip_mark_positions = {});

// Per candidate, in order: STATUS_OK and nothing stands ahead of it in its feed column. The
// campaign's skip-mark lift reads this (SurveyTray.Result.overview_candidate_feed_front), so a
// lift is charged only for a skipped product that would be a front once lifted (Card 066 × 058).
[[nodiscard]] std::vector<bool> feed_column_fronts(
  const std::vector<TrayObservation> & candidates, const FeedOrder & feed);

enum class ConfirmationVerdict : std::uint8_t
{
  // The close view saw the candidate and its identity agrees: this observation replaces the
  // hypothesis and is the only verdict that authorizes anything.
  kConfirmed,
  // A product stands at the candidate's place, but it is a different class.
  kRefutedClassMismatch,
  // Same class, different SKU.
  kRefutedSkuMismatch,
  // The close view saw observations, but none at the candidate's place: it is not there.
  kRefutedAbsent,
  // Nothing arrived after the arm stopped. The survey did not look; that is not a refutation.
  kNoObservation,
};

[[nodiscard]] const char * confirmation_verdict_name(ConfirmationVerdict verdict) noexcept;

struct ConfirmationEvaluation
{
  ConfirmationVerdict verdict{ConfirmationVerdict::kNoObservation};
  // The associated observation when one landed inside the association radius.
  std::optional<TrayObservation> associated_observation;
  // The nearest observation when any arrived, associated or not. Set for the absent verdict so
  // a report can say what the camera did see.
  std::optional<TrayObservation> nearest_observation;
  double associated_distance_m{0.0};
  std::string detail;
};

// Judge the close view against the selected candidate. `observations` must already be the
// confirm window: confirmation backend, status OK, stamped after the arm stopped.
[[nodiscard]] ConfirmationEvaluation evaluate_confirmation(
  const TrayObservation & selected, const std::vector<TrayObservation> & observations,
  const TraySurveyLogicConfig & config);

// The zero-dwell drain's verdict for one station (Card 050's drain, Milestone 10 Stage 4): read
// from the drained report's own stage counts, so the log line names the stage that produced the
// zero instead of guessing from arrival timing.
[[nodiscard]] const char * classify_drained_station(
  const restocker_interfaces::msg::TrayStationAcquisition & report) noexcept;

// Milestone 10 §6, Card 069: what a completed overview says about one place where the caller
// believes a tray product stands. Values match SurveyTray.Result.PROBE_*. Only kViewedEmpty is
// evidence of absence; every other verdict leaves the caller's belief unchanged or confirms it.
enum class AbsenceProbeVerdict : std::uint8_t
{
  kNotEvaluated = 0,
  // An overview candidate of any class lies within the merge radius: something is there.
  kSeen = 1,
  // A healthy station framed the place with a clear line of sight and saw nothing there.
  kViewedEmpty = 2,
  // Framed by a healthy station, but a nearer point blocks every such line of sight.
  kOccluded = 3,
  // No visited station frames the place.
  kNotCovered = 4,
  // Framed only by stations whose acquisition was not healthy.
  kNoEvidence = 5,
};

[[nodiscard]] const char * absence_probe_verdict_name(AbsenceProbeVerdict verdict) noexcept;

// Whether a station's acquisition demonstrably worked, so an empty place in its view can be
// read as empty: both taps configured, images and detector frames inside the window, proposals
// in some frame, and at least one admitted observation. A station that detected nothing at all
// (a blind detector looks exactly like an empty tray) or admitted nothing is never evidence.
[[nodiscard]] bool station_acquisition_healthy(
  const restocker_interfaces::msg::TrayStationAcquisition & report) noexcept;

// One visited overview station as the probe classification sees it.
struct AbsenceStationView
{
  // planning frame <- camera optical frame, at the station's nominal pose.
  Eigen::Isometry3d world_from_optical{Eigen::Isometry3d::Identity()};
  bool healthy{false};
};

struct AbsenceProbeConfig
{
  restocker_perception::SensorFrustum frustum;
  // Nothing within this radius of the probe may have been seen. Wider than the merge radius on
  // purpose: an overview estimate lands within about 11 mm of a stationary product (SC-004
  // evaluator), while a product disturbed since its last confirm view may sit centimetres off.
  double seen_radius_m{0.10};
  // A point nearer the camera within this distance of the line of sight blocks the view.
  double occluder_radius_m{0.05};
  // The probe must project at least this far inside every image edge.
  double framing_margin_px{20.0};
};

// Classify one probe (planning frame) against a completed overview. `candidates` are the
// merged overview candidates' positions; `occluders` are every other point that could stand in
// the line of sight (the other probes and the candidates), excluding the probe itself.
[[nodiscard]] AbsenceProbeVerdict classify_absence_probe(
  const Eigen::Vector3d & probe, const std::vector<Eigen::Vector3d> & candidates,
  const std::vector<Eigen::Vector3d> & occluders,
  const std::vector<AbsenceStationView> & stations, const AbsenceProbeConfig & config);

}  // namespace restocker_task_executor
