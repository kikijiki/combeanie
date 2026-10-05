// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "restocker_perception/perception_port.hpp"

namespace restocker_perception
{

// Per-gate proposal drops from the most recent estimate() call (Card 050). Every count is a
// blob detect() proposed that estimate() refused without publishing — the stage that is silent
// by design ("nothing records that a proposal was refused"), and the one a zero-frame dwell has
// to name. Cleared at the start of each estimate(); all zero when nothing was refused.
struct EstimateDropCounts
{
  // The mask's usable points (depth band, workspace) were too sparse to fit anything.
  std::uint32_t too_few_points{0U};
  // The blob's visible vertical extent is taller than the product it claims to be (two things,
  // or something standing behind something else).
  std::uint32_t extent{0U};
  // The world-Z top band held fewer points than a top face needs.
  std::uint32_t top_face_points{0U};
  // Radius ratios outside the catalogue bands: not a cylinder of this radius (a container on
  // its side lands here, which is why a tipped product publishes nothing).
  std::uint32_t radial_profile{0U};
  // The axis-fit quality gate refused the hull (same refusals last_axis_fit_refusal reports).
  std::uint32_t axis_fit{0U};
  // The fit's centre was not finite.
  std::uint32_t nonfinite_centre{0U};
  // Card 063: a split band's discs were admitted but stood closer than two catalogued radii,
  // which two upright products cannot; the whole component was refused.
  std::uint32_t split_overlap{0U};

  [[nodiscard]] std::uint32_t total() const noexcept
  {
    return too_few_points + extent + top_face_points + radial_profile + axis_fit + split_overlap +
           nonfinite_centre;
  }
};

// Card 065, diagnostic only: what estimate() did with one proposal, and why. Recorded only while
// diagnostics are enabled (set_diagnostics_enabled); recording never changes a decision.
struct ProposalDiagnostic
{
  std::uint8_t product_class{0U};
  // The detection's pixel box, half-open, as detect() published it.
  std::uint16_t x_min{0U};
  std::uint16_t y_min{0U};
  std::uint16_t x_max{0U};
  std::uint16_t y_max{0U};
  std::uint32_t mask_pixels{0U};
  // Mask pixels with a depth in the band that deproject inside the workspace.
  std::uint32_t points{0U};
  // Planning-frame mean of those points; not finite when there are none.
  Eigen::Vector3d centroid{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  double top_height{std::numeric_limits<double>::quiet_NaN()};
  double visible_extent{std::numeric_limits<double>::quiet_NaN()};
  std::uint32_t band_points{0U};
  // Discs the top band split into (0 when the split is off or the band stayed whole).
  std::uint32_t discs{0U};
  // "published", "published (set_aside)", or the EstimateDropCounts field that refused the
  // proposal.
  std::string verdict;
  // For a split band with a failing disc: the Card 065 decision and its minimum clearance.
  std::string set_aside_answer;
  double minimum_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  // Per disc (or the whole band): where it stood, how many points, the radial ratios, and the
  // gate that answered.
  std::string detail;
};

// Card 065, diagnostic only: a same-class colour component detect() dropped for being smaller
// than minimum_component_pixels. It never becomes a detection, so nothing else records it.
struct SmallComponentDiagnostic
{
  std::uint8_t product_class{0U};
  std::uint32_t pixels{0U};
  std::uint16_t x_min{0U};
  std::uint16_t y_min{0U};
  std::uint16_t x_max{0U};
  std::uint16_t y_max{0U};
};

// Card 065, diagnostic only: how detect() classified the pixels of the image's central square,
// where a confirm view aims its product. A product whose pixels never classify leaves no
// proposal and no small component, so only this says what the camera saw there.
struct CentreCensus
{
  std::uint32_t pixels{0U};
  std::uint32_t outside_depth_band{0U};
  std::uint32_t too_dark{0U};
  std::uint32_t no_signature{0U};
  // Pixels classified as each configured signature, in configuration order.
  std::vector<std::uint32_t> classified;
  // Means over the pixels with a depth in the band.
  double mean_red{0.0};
  double mean_green{0.0};
  double mean_blue{0.0};
  double mean_depth_m{0.0};
};

// Card 065 (Milestone 10 §6, "A column's partly hidden far end does not refuse its front"): one
// disc of a split top band after every gate judged it, in world XY.
struct JudgedDisc
{
  std::vector<Eigen::Vector2d> points;
  bool admitted{false};
  // The fitted axis; meaningful only for an admitted disc.
  Eigen::Vector2d fitted_centre{Eigen::Vector2d::Zero()};
};

// The answer for a split component's discs, naming the condition that decided it.
enum class SetAsideAnswer : std::uint8_t
{
  kSetAside,
  kNothingFailed,
  kNothingAdmitted,
  // Condition 2: a failing disc is not farther from the camera than every admitted disc.
  kFailingDiscNearer,
  // Condition 3: a failing-disc point lies within 1.5 catalogued radii of an admitted centre.
  kInsideAdmittedFootprint,
  // An empty disc, a non-finite position or a non-positive radius.
  kInvalid,
};

[[nodiscard]] const char * set_aside_answer_name(SetAsideAnswer answer) noexcept;

struct SetAsideDecision
{
  SetAsideAnswer answer{SetAsideAnswer::kInvalid};
  // Smallest horizontal distance from a failing-disc point to an admitted fitted centre, when
  // both exist (condition 3 needs it to exceed 1.5 catalogued radii); NaN otherwise.
  double minimum_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  [[nodiscard]] bool set_aside() const noexcept {return answer == SetAsideAnswer::kSetAside;}
};

// Whether the failing discs of a split component may be set aside rather than refuse it: at
// least one disc is admitted, every failing disc's point centroid is horizontally farther from
// the camera than every admitted disc's, and every failing-disc point lies more than 1.5
// catalogued radii from every admitted fitted centre.
[[nodiscard]] SetAsideDecision judge_failed_discs(
  const std::vector<JudgedDisc> & discs, const Eigen::Vector2d & camera_xy,
  double catalogued_radius_m);

// judge_failed_discs(...).set_aside(): false when nothing failed.
[[nodiscard]] bool failed_discs_can_be_set_aside(
  const std::vector<JudgedDisc> & discs, const Eigen::Vector2d & camera_xy,
  double catalogued_radius_m);

// One product category the backend knows how to look for.
//
// The reference colour is the colour the product renders as in this workcell's lighting, not the
// albedo its material declares: the two differ by ambient lift and, for the translucent bottles,
// by whatever is behind them.
//
// nominal_radius_m and nominal_height_m are the catalogued cylinder dimensions. They are the model
// this backend fits: the height converts an observed top face into a centre, and the radius gates
// a product from anything else of the same colour.
struct ProductSignature
{
  std::uint8_t product_class{restocker_interfaces::msg::ObjectDetection::PRODUCT_CLASS_UNKNOWN};
  // The catalogued stock-keeping unit this appearance belongs to. A signature describes one
  // product variant, not a category: the three products in this cell are three appearances and
  // three SKUs.
  //
  // Required. Lane policy admits a product by class and SKU, so an observation with no SKU would
  // be refused at the destination lane, far from where the evidence went missing.
  std::string sku;
  std::array<double, 3> reference_rgb{0.0, 0.0, 0.0};
  double nominal_radius_m{0.0};
  double nominal_height_m{0.0};
};

// An axis-aligned box in the planning frame, outside which a detection is not a product this cell
// contains.
struct WorkspaceBounds
{
  Eigen::Vector3d minimum{Eigen::Vector3d::Zero()};
  Eigen::Vector3d maximum{Eigen::Vector3d::Zero()};

  [[nodiscard]] bool contains(const Eigen::Vector3d & point) const noexcept
  {
    return (point.array() >= minimum.array()).all() && (point.array() <= maximum.array()).all();
  }
};

struct ColourDepthConfig
{
  std::string backend_name{"overhead_rgbd_colour_depth"};
  std::string backend_version{"1.0.0"};
  std::vector<ProductSignature> signatures;
  WorkspaceBounds workspace;

  // Chromaticity is the pixel's colour divided by its own intensity, so shading does not move a
  // pixel's class. This is the L2 radius in that space within which a pixel is accepted as a
  // signature's colour.
  double chromaticity_tolerance{0.055};
  // Below this channel sum a pixel is dark enough that its chromaticity is quantization noise.
  double minimum_intensity{60.0};
  // Depth returns outside the sensor's usable band are not measurements.
  double minimum_depth_m{0.20};
  double maximum_depth_m{8.0};
  // A connected region smaller than this is not one of these products at this camera's range.
  std::uint32_t minimum_component_pixels{40};

  // Pose estimation. The top face of an upright cylinder is a full disc from a camera looking
  // down, but at this workcell's 51-degree tray elevation the world-Z band that selects it also
  // admits a strip of the camera-facing barrel, and the disc is sampled more densely on the near
  // rim — a plain XY mean is therefore pulled toward the camera (Card 036). These select the top
  // face out of the component's points; estimate() then takes the cylinder axis from an
  // algebraic circle fit to the band's convex hull.
  //
  // Rim-circle assumption: every hull vertex must lie on the product's world-XY rim circle —
  // true for an upright axisymmetric product's rim and lateral surface, false when an occluder's
  // straight cut crosses the disc interior or two same-colour products merge. The fit carries a
  // quality gate (span, rank, centre plausibility, geometric residual); a failed gate refuses
  // the detection instead of publishing a wrong centre.
  double top_face_band_m{0.02};
  double top_face_percentile{0.92};
  std::uint32_t minimum_top_face_points{15};
  // Card 063: when positive, a top band that separates into two or more discs (world-XY gaps
  // wider than this) is judged disc by disc, all or nothing. Zero keeps the band whole, which is
  // what every duty did before and what the overhead pipeline still does.
  double top_face_split_gap_m{0.0};

  // The measured radial spread of the top-face points, as a multiple of the signature's nominal
  // radius. A disc sampled by a downward-looking camera has a mean radius of about 0.68 R and a
  // maximum of about 1.05 R; anything far outside that is not a cylinder of this radius.
  double minimum_mean_radius_ratio{0.45};
  double maximum_mean_radius_ratio{0.95};
  double minimum_extreme_radius_ratio{0.70};
  double maximum_extreme_radius_ratio{1.35};
  // A component whose visible vertical extent exceeds the product's own height is two things.
  double height_tolerance_m{0.03};

  // Floor on the reported translation standard deviation, in metres. This is an accuracy floor
  // in the sense of Milestone 10 §7 (Card 035): the covariance feeds the attachment fidelity
  // budget, so the claim must cover measured error against ground truth — including systematic
  // offset — not only the point-scatter precision of the top-face fit (which is sub-millimetre
  // with hundreds of points and smaller than extrinsics and depth quantization support). Each
  // duty's shipped value is calibrated against that duty's measured confirm/overview error
  // distribution; see the yaml comment where the value is set.
  double translation_sigma_floor_m{0.003};
};

// A classical RGB-D detector and pose estimator for known, brightly coloured, upright cylinders.
//
// Not a pretrained network. It implements the two backend-neutral ports so a learned model can
// replace it without any consumer changing.
//
// detect() is pure image and depth work and needs no extrinsics. Every judgement that needs to
// know where the camera is (the workspace bound, the cylinder fit, the uprightness check) is in
// estimate(), which is handed the transform explicitly.
//
// Colour proposes and geometry disposes: a blob of the right colour that is not a cylinder of the
// right radius in the right place yields no observation. The robot's own blue links classify as
// the small bottle by colour; the radius gate rejects them.
class ColourDepthBackend final : public DetectionPort, public PoseEstimationPort
{
public:
  [[nodiscard]] static Result<ColourDepthBackend> create(ColourDepthConfig config);

  [[nodiscard]] restocker_interfaces::msg::PerceptionFrame detect(const RgbdFrame & frame) override;

  [[nodiscard]] Result<std::vector<restocker_interfaces::msg::ObjectObservation>> estimate(
    const restocker_interfaces::msg::PerceptionFrame & detections, const RgbdFrame & frame,
    const FramedTransform & camera_to_planning) override;

  [[nodiscard]] const ColourDepthConfig & config() const noexcept {return config_;}

  // Axis-fit quality-gate refusals from the most recent estimate() call (Card 036 review
  // follow-up). The backend has no node handle, so it stashes the first reason and the count;
  // perception_node emits them through its throttled logger. Cleared at the start of each
  // estimate(); both are empty/zero when nothing was refused.
  [[nodiscard]] const std::string & last_axis_fit_refusal() const noexcept
  {
    return axis_fit_refusal_reason_;
  }
  [[nodiscard]] std::size_t last_axis_fit_refusal_count() const noexcept
  {
    return axis_fit_refusal_count_;
  }
  [[nodiscard]] const EstimateDropCounts & last_estimate_drop_counts() const noexcept
  {
    return drop_counts_;
  }

  // Card 065, diagnostic only. Off by default. While enabled, detect() records the components it
  // drops for size and estimate() records one ProposalDiagnostic per detection it judges; each
  // call clears its own list first. Decisions are identical either way.
  void set_diagnostics_enabled(bool enabled) noexcept {diagnostics_enabled_ = enabled;}
  [[nodiscard]] bool diagnostics_enabled() const noexcept {return diagnostics_enabled_;}
  [[nodiscard]] const std::vector<ProposalDiagnostic> & last_proposal_diagnostics() const noexcept
  {
    return proposal_diagnostics_;
  }
  [[nodiscard]] const std::vector<SmallComponentDiagnostic> & last_small_components()
  const noexcept
  {
    return small_components_;
  }
  // Half-width of the central square, in pixels (the square is 2 * this on a side).
  static constexpr std::size_t kCentreCensusHalfWidthPx = 20U;
  [[nodiscard]] const CentreCensus & last_centre_census() const noexcept {return centre_census_;}

private:
  explicit ColourDepthBackend(ColourDepthConfig config);

  // Every geometry gate after the top band is chosen, applied to one disc's points: the radial
  // profile, the axis-fit quality gate and a finite centre. Counts its refusal in drop_counts_.
  [[nodiscard]] std::optional<restocker_interfaces::msg::ObjectObservation> estimate_disc(
    const std::vector<Eigen::Vector3d> & top_face, double top_height,
    const ProductSignature & signature,
    const restocker_interfaces::msg::ObjectDetection & detection,
    const builtin_interfaces::msg::Time & stamp, const std::string & planning_frame,
    double coverage, std::string * why = nullptr);

  ColourDepthConfig config_;
  std::vector<Eigen::Vector3d> chromaticities_;
  std::string axis_fit_refusal_reason_;
  std::size_t axis_fit_refusal_count_{0U};
  EstimateDropCounts drop_counts_;
  bool diagnostics_enabled_{false};
  std::vector<ProposalDiagnostic> proposal_diagnostics_;
  std::vector<SmallComponentDiagnostic> small_components_;
  CentreCensus centre_census_;
};

}  // namespace restocker_perception
