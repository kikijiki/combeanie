// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/colour_depth_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "restocker_perception/axis_fit.hpp"

namespace restocker_perception
{
namespace
{

using DetectionMessage = restocker_interfaces::msg::ObjectDetection;
using ObservationMessage = restocker_interfaces::msg::ObjectObservation;
using PerceptionFrameMessage = restocker_interfaces::msg::PerceptionFrame;

constexpr std::size_t kNoClass = std::numeric_limits<std::size_t>::max();

// Groups top-band points into discs (Card 063): connected components of the occupied cells of a
// world-XY grid of `gap_m` cells, neighbours including diagonals, so points closer than `gap_m`
// always share a disc, and discs separated by more than two cells along an axis (about 2.8
// diagonally) never do. Gaps in between may go either way, which only ever keeps discs joined,
// and so refused. Groups smaller than `minimum_points` are not discs and are not
// returned: when two or more real discs remain, such fragments are ignored rather than judged
// (Card 063 review N6); every returned disc still passes every gate.
[[nodiscard]] std::vector<std::vector<Eigen::Vector3d>> split_into_discs(
  const std::vector<Eigen::Vector3d> & band, double gap_m, std::size_t minimum_points)
{
  using Cell = std::pair<std::int64_t, std::int64_t>;
  std::map<Cell, std::vector<std::size_t>> cells;
  for (std::size_t index = 0; index < band.size(); ++index) {
    cells[Cell{
        static_cast<std::int64_t>(std::floor(band[index].x() / gap_m)),
        static_cast<std::int64_t>(std::floor(band[index].y() / gap_m))}].push_back(index);
  }
  std::map<Cell, bool> visited;
  std::vector<std::vector<Eigen::Vector3d>> discs;
  for (const auto & entry : cells) {
    if (visited[entry.first]) {
      continue;
    }
    visited[entry.first] = true;
    std::vector<Eigen::Vector3d> disc;
    std::vector<Cell> pending{entry.first};
    while (!pending.empty()) {
      const Cell cell = pending.back();
      pending.pop_back();
      for (const std::size_t index : cells.at(cell)) {
        disc.push_back(band[index]);
      }
      for (std::int64_t dx = -1; dx <= 1; ++dx) {
        for (std::int64_t dy = -1; dy <= 1; ++dy) {
          const Cell neighbour{cell.first + dx, cell.second + dy};
          if (cells.contains(neighbour) && !visited[neighbour]) {
            visited[neighbour] = true;
            pending.push_back(neighbour);
          }
        }
      }
    }
    if (disc.size() >= minimum_points) {
      discs.push_back(std::move(disc));
    }
  }
  return discs;
}

// Card 065, diagnostic only: which EstimateDropCounts field moved between two snapshots.
[[nodiscard]] const char * drop_stage(
  const EstimateDropCounts & before, const EstimateDropCounts & after)
{
  if (after.too_few_points != before.too_few_points) {return "too_few_points";}
  if (after.extent != before.extent) {return "extent";}
  if (after.top_face_points != before.top_face_points) {return "top_face_points";}
  if (after.radial_profile != before.radial_profile) {return "radial_profile";}
  if (after.axis_fit != before.axis_fit) {return "axis_fit";}
  if (after.nonfinite_centre != before.nonfinite_centre) {return "nonfinite_centre";}
  if (after.split_overlap != before.split_overlap) {return "split_overlap";}
  return "published";
}

template<typename Value>
[[nodiscard]] Result<Value> failure(PerceptionErrorCode code, std::string detail)
{
  return Result<Value>::failure(PerceptionError{code, std::move(detail)});
}

// Per-pixel classification against the signatures, and how well it held up.
struct Classification
{
  std::vector<std::size_t> label;      // signature index, or kNoClass
  std::vector<float> posterior;        // confidence that the label is right
};

[[nodiscard]] double sample_depth(const sensor_msgs::msg::Image & depth, std::size_t index)
{
  float value = 0.0F;
  std::memcpy(&value, depth.data.data() + index * sizeof(float), sizeof(float));
  return static_cast<double>(value);
}

// The posterior that a pixel belongs to its nearest signature rather than to another signature or
// to none of them. The "none of them" hypothesis sits at exactly the acceptance tolerance, so a
// pixel on the tolerance boundary scores at most 0.5; above 0.5 it is strictly inside the colour
// the signature describes.
[[nodiscard]] float class_posterior(
  const std::vector<double> & squared_distances, std::size_t best, double tolerance)
{
  const double sigma = tolerance / 2.0;
  const double scale = 2.0 * sigma * sigma;
  double total = std::exp(-(tolerance * tolerance) / scale);
  for (const double squared : squared_distances) {
    total += std::exp(-squared / scale);
  }
  const double numerator = std::exp(-squared_distances[best] / scale);
  return static_cast<float>(numerator / total);
}

}  // namespace

ColourDepthBackend::ColourDepthBackend(ColourDepthConfig config)
: config_(std::move(config))
{
  chromaticities_.reserve(config_.signatures.size());
  for (const auto & signature : config_.signatures) {
    const Eigen::Vector3d colour(
      signature.reference_rgb[0], signature.reference_rgb[1], signature.reference_rgb[2]);
    chromaticities_.push_back(colour / colour.sum());
  }
}

Result<ColourDepthBackend> ColourDepthBackend::create(ColourDepthConfig config)
{
  if (config.backend_name.empty() || config.backend_version.empty()) {
    return failure<ColourDepthBackend>(
      PerceptionErrorCode::InvalidArgument, "backend name and version must not be empty");
  }
  if (config.signatures.empty()) {
    return failure<ColourDepthBackend>(
      PerceptionErrorCode::InvalidArgument, "at least one product signature is required");
  }
  for (const auto & signature : config.signatures) {
    const double intensity =
      signature.reference_rgb[0] + signature.reference_rgb[1] + signature.reference_rgb[2];
    if (!std::isfinite(intensity) || intensity <= 0.0) {
      return failure<ColourDepthBackend>(
        PerceptionErrorCode::InvalidArgument, "a signature reference colour is not usable");
    }
    if (signature.sku.empty()) {
      return failure<ColourDepthBackend>(
        PerceptionErrorCode::InvalidArgument, "a signature does not name the SKU it identifies");
    }
    if (!(signature.nominal_radius_m > 0.0) || !(signature.nominal_height_m > 0.0)) {
      return failure<ColourDepthBackend>(
        PerceptionErrorCode::InvalidArgument, "signature dimensions must be positive");
    }
    if (signature.product_class == DetectionMessage::PRODUCT_CLASS_UNKNOWN ||
      signature.product_class > DetectionMessage::PRODUCT_CLASS_LARGE_BOTTLE)
    {
      return failure<ColourDepthBackend>(
        PerceptionErrorCode::InvalidArgument, "signature product class is not a known category");
    }
  }
  if (config.minimum_component_pixels == 0U || config.minimum_top_face_points == 0U) {
    return failure<ColourDepthBackend>(
      PerceptionErrorCode::InvalidArgument,
      "minimum component and top-face sample counts must be positive, or a detection with no "
      "measurable geometry would be reported as a pose");
  }
  if (!(config.chromaticity_tolerance > 0.0) || !(config.minimum_depth_m > 0.0) ||
    !(config.maximum_depth_m > config.minimum_depth_m) ||
    !(config.top_face_band_m > 0.0) ||
    !(config.top_face_percentile > 0.0) || !(config.top_face_percentile <= 1.0) ||
    !(config.translation_sigma_floor_m > 0.0) ||
    !std::isfinite(config.top_face_split_gap_m) || config.top_face_split_gap_m < 0.0)
  {
    return failure<ColourDepthBackend>(
      PerceptionErrorCode::InvalidArgument, "backend thresholds are not in their valid ranges");
  }
  if (!(config.workspace.minimum.array() < config.workspace.maximum.array()).all()) {
    return failure<ColourDepthBackend>(
      PerceptionErrorCode::InvalidArgument, "workspace bounds are empty or inverted");
  }
  return Result<ColourDepthBackend>::success(ColourDepthBackend(std::move(config)));
}

const char * set_aside_answer_name(const SetAsideAnswer answer) noexcept
{
  switch (answer) {
    case SetAsideAnswer::kSetAside:
      return "set_aside";
    case SetAsideAnswer::kNothingFailed:
      return "nothing_failed";
    case SetAsideAnswer::kNothingAdmitted:
      return "nothing_admitted";
    case SetAsideAnswer::kFailingDiscNearer:
      return "failing_disc_nearer";
    case SetAsideAnswer::kInsideAdmittedFootprint:
      return "inside_admitted_footprint";
    case SetAsideAnswer::kInvalid:
      return "invalid";
  }
  return "invalid";
}

SetAsideDecision judge_failed_discs(
  const std::vector<JudgedDisc> & discs, const Eigen::Vector2d & camera_xy,
  const double catalogued_radius_m)
{
  SetAsideDecision decision;
  if (!(catalogued_radius_m > 0.0) || !camera_xy.allFinite()) {
    return decision;
  }
  bool any_admitted = false;
  bool any_failing = false;
  double farthest_admitted = 0.0;
  double nearest_failing = std::numeric_limits<double>::infinity();
  for (const auto & disc : discs) {
    if (disc.points.empty()) {
      return decision;
    }
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto & point : disc.points) {
      centroid += point;
    }
    centroid /= static_cast<double>(disc.points.size());
    const double range = (centroid - camera_xy).norm();
    if (!std::isfinite(range) || (disc.admitted && !disc.fitted_centre.allFinite())) {
      return decision;
    }
    if (disc.admitted) {
      any_admitted = true;
      farthest_admitted = std::max(farthest_admitted, range);
    } else {
      any_failing = true;
      nearest_failing = std::min(nearest_failing, range);
    }
  }
  if (!any_failing) {
    decision.answer = SetAsideAnswer::kNothingFailed;
    return decision;
  }
  if (!any_admitted) {
    decision.answer = SetAsideAnswer::kNothingAdmitted;
    return decision;
  }
  // Recorded for every component with both kinds of disc, whichever condition decides it.
  double clearance = std::numeric_limits<double>::infinity();
  for (const auto & failing : discs) {
    if (failing.admitted) {
      continue;
    }
    for (const auto & admitted : discs) {
      if (!admitted.admitted) {
        continue;
      }
      for (const auto & point : failing.points) {
        clearance = std::min(clearance, (point - admitted.fitted_centre).norm());
      }
    }
  }
  decision.minimum_clearance_m = clearance;
  // Only something nearer, or the image's own edge beyond it, can cut a disc short.
  if (!(nearest_failing > farthest_admitted)) {
    decision.answer = SetAsideAnswer::kFailingDiscNearer;
    return decision;
  }
  // A failing disc reaching into an admitted product's own footprint may be that product's own
  // top or barrel strip (a tilted product's fragment): it keeps refusing the component.
  if (!(clearance > 1.5 * catalogued_radius_m)) {
    decision.answer = SetAsideAnswer::kInsideAdmittedFootprint;
    return decision;
  }
  decision.answer = SetAsideAnswer::kSetAside;
  return decision;
}

bool failed_discs_can_be_set_aside(
  const std::vector<JudgedDisc> & discs, const Eigen::Vector2d & camera_xy,
  const double catalogued_radius_m)
{
  return judge_failed_discs(discs, camera_xy, catalogued_radius_m).set_aside();
}

PerceptionFrameMessage ColourDepthBackend::detect(const RgbdFrame & frame)
{
  PerceptionFrameMessage message;
  // The acquisition is the only authority for when and where these detections are valid.
  message.header.frame_id = frame.frame_id();
  message.header.stamp = frame.stamp();
  message.image_width = frame.intrinsics().width;
  message.image_height = frame.intrinsics().height;
  message.backend_name = config_.backend_name;
  message.backend_version = config_.backend_version;
  message.status = PerceptionFrameMessage::STATUS_OK;

  const auto & colour = frame.colour();
  const auto & depth = frame.depth();
  if (colour.encoding != "rgb8" || depth.encoding != "32FC1") {
    message.status = PerceptionFrameMessage::STATUS_INVALID_FRAME;
    message.status_detail =
      "expected rgb8 colour and 32FC1 depth, got '" + colour.encoding + "' and '" +
      depth.encoding + "'";
    return message;
  }
  const std::size_t width = colour.width;
  const std::size_t height = colour.height;
  if (colour.step < width * 3U || depth.step < width * sizeof(float) ||
    depth.step % sizeof(float) != 0U ||
    colour.data.size() < colour.step * height || depth.data.size() < depth.step * height)
  {
    message.status = PerceptionFrameMessage::STATUS_INVALID_FRAME;
    message.status_detail =
      "image buffers are shorter than their declared geometry, or the depth row stride is not a "
      "whole number of samples";
    return message;
  }

  small_components_.clear();
  centre_census_ = CentreCensus{};
  centre_census_.classified.assign(config_.signatures.size(), 0U);
  const std::size_t centre_column = width / 2U;
  const std::size_t centre_row = height / 2U;
  const auto in_centre = [&](std::size_t row, std::size_t column) {
    return diagnostics_enabled_ &&
           row + kCentreCensusHalfWidthPx >= centre_row &&
           row < centre_row + kCentreCensusHalfWidthPx &&
           column + kCentreCensusHalfWidthPx >= centre_column &&
           column < centre_column + kCentreCensusHalfWidthPx;
  };
  std::uint32_t census_in_band = 0U;
  Classification classified;
  classified.label.assign(width * height, kNoClass);
  classified.posterior.assign(width * height, 0.0F);

  std::vector<double> squared(chromaticities_.size(), 0.0);
  const double tolerance_squared = config_.chromaticity_tolerance * config_.chromaticity_tolerance;
  for (std::size_t row = 0; row < height; ++row) {
    const std::uint8_t * colour_row = colour.data.data() + row * colour.step;
    for (std::size_t column = 0; column < width; ++column) {
      const std::size_t index = row * width + column;
      const bool census = in_centre(row, column);
      if (census) {
        ++centre_census_.pixels;
      }
      const double metres = sample_depth(depth, row * (depth.step / sizeof(float)) + column);
      if (!std::isfinite(metres) || metres < config_.minimum_depth_m ||
        metres > config_.maximum_depth_m)
      {
        if (census) {
          ++centre_census_.outside_depth_band;
        }
        continue;
      }
      const double red = colour_row[column * 3U];
      const double green = colour_row[column * 3U + 1U];
      const double blue = colour_row[column * 3U + 2U];
      if (census) {
        ++census_in_band;
        centre_census_.mean_red += red;
        centre_census_.mean_green += green;
        centre_census_.mean_blue += blue;
        centre_census_.mean_depth_m += metres;
      }
      const double intensity = red + green + blue;
      if (intensity < config_.minimum_intensity) {
        if (census) {
          ++centre_census_.too_dark;
        }
        continue;
      }
      const Eigen::Vector3d chromaticity(red / intensity, green / intensity, blue / intensity);
      std::size_t best = 0;
      double best_squared = std::numeric_limits<double>::max();
      for (std::size_t signature = 0; signature < chromaticities_.size(); ++signature) {
        squared[signature] = (chromaticity - chromaticities_[signature]).squaredNorm();
        if (squared[signature] < best_squared) {
          best_squared = squared[signature];
          best = signature;
        }
      }
      if (best_squared > tolerance_squared) {
        if (census) {
          ++centre_census_.no_signature;
        }
        continue;
      }
      if (census) {
        ++centre_census_.classified[best];
      }
      classified.label[index] = best;
      classified.posterior[index] =
        class_posterior(squared, best, config_.chromaticity_tolerance);
    }
  }

  if (census_in_band > 0U) {
    const auto samples = static_cast<double>(census_in_band);
    centre_census_.mean_red /= samples;
    centre_census_.mean_green /= samples;
    centre_census_.mean_blue /= samples;
    centre_census_.mean_depth_m /= samples;
  }

  // Four-connected components within a class. A blob split by an occluder becomes two detections
  // rather than one spanning the gap; the geometry gate in estimate() then rejects whichever
  // fragment is not a whole product.
  std::vector<std::uint8_t> visited(width * height, 0U);
  std::vector<std::size_t> pending;
  std::vector<std::size_t> component;
  for (std::size_t seed = 0; seed < classified.label.size(); ++seed) {
    if (visited[seed] != 0U || classified.label[seed] == kNoClass) {
      continue;
    }
    const std::size_t signature = classified.label[seed];
    component.clear();
    pending.assign(1, seed);
    visited[seed] = 1U;
    while (!pending.empty()) {
      const std::size_t index = pending.back();
      pending.pop_back();
      component.push_back(index);
      const std::size_t row = index / width;
      const std::size_t column = index % width;
      const auto push = [&](std::size_t neighbour) {
        if (visited[neighbour] == 0U && classified.label[neighbour] == signature) {
          visited[neighbour] = 1U;
          pending.push_back(neighbour);
        }
      };
      if (column + 1U < width) {push(index + 1U);}
      if (column > 0U) {push(index - 1U);}
      if (row + 1U < height) {push(index + width);}
      if (row > 0U) {push(index - width);}
    }
    if (component.size() < config_.minimum_component_pixels) {
      if (diagnostics_enabled_) {
        SmallComponentDiagnostic small;
        small.product_class = config_.signatures[signature].product_class;
        small.pixels = static_cast<std::uint32_t>(component.size());
        small.x_min = static_cast<std::uint16_t>(width);
        small.y_min = static_cast<std::uint16_t>(height);
        for (const std::size_t index : component) {
          const auto row = static_cast<std::uint16_t>(index / width);
          const auto column = static_cast<std::uint16_t>(index % width);
          small.x_min = std::min(small.x_min, column);
          small.y_min = std::min(small.y_min, row);
          small.x_max = std::max(small.x_max, static_cast<std::uint16_t>(column + 1U));
          small.y_max = std::max(small.y_max, static_cast<std::uint16_t>(row + 1U));
        }
        small_components_.push_back(small);
      }
      continue;
    }

    std::size_t x_min = width;
    std::size_t x_max = 0;
    std::size_t y_min = height;
    std::size_t y_max = 0;
    double posterior_total = 0.0;
    for (const std::size_t index : component) {
      const std::size_t row = index / width;
      const std::size_t column = index % width;
      x_min = std::min(x_min, column);
      x_max = std::max(x_max, column);
      y_min = std::min(y_min, row);
      y_max = std::max(y_max, row);
      posterior_total += classified.posterior[index];
    }

    DetectionMessage detection;
    detection.x_min = static_cast<std::uint16_t>(x_min);
    detection.y_min = static_cast<std::uint16_t>(y_min);
    detection.x_max = static_cast<std::uint16_t>(x_max + 1U);
    detection.y_max = static_cast<std::uint16_t>(y_max + 1U);
    detection.product_class = config_.signatures[signature].product_class;
    detection.score = static_cast<float>(posterior_total / static_cast<double>(component.size()));
    // No cross-frame image-space identity is established here; identity is assigned from the
    // estimated pose after this stage and travels on ObjectObservation.source_object_id. 0 is
    // ObjectDetection's value for an untracked detection.
    detection.instance_id = 0U;
    detection.has_mask = true;

    const std::size_t box_width = x_max + 1U - x_min;
    const std::size_t box_height = y_max + 1U - y_min;
    detection.mask.width = static_cast<std::uint16_t>(box_width);
    detection.mask.height = static_cast<std::uint16_t>(box_height);
    detection.mask.score = detection.score;
    // Run lengths over the box in row-major order, starting with a background run so the parity
    // of a run's position is its class. A leading zero-length run means the box opens on
    // foreground.
    std::vector<std::uint8_t> membership(box_width * box_height, 0U);
    for (const std::size_t index : component) {
      const std::size_t row = index / width - y_min;
      const std::size_t column = index % width - x_min;
      membership[row * box_width + column] = 1U;
    }
    std::uint8_t current = 0U;
    std::uint32_t run = 0U;
    for (const std::uint8_t pixel : membership) {
      if (pixel == current) {
        ++run;
        continue;
      }
      detection.mask.rle_counts.push_back(run);
      current = pixel;
      run = 1U;
    }
    detection.mask.rle_counts.push_back(run);

    message.detections.push_back(std::move(detection));
  }

  return message;
}

Result<std::vector<ObservationMessage>> ColourDepthBackend::estimate(
  const PerceptionFrameMessage & detections, const RgbdFrame & frame,
  const FramedTransform & camera_to_planning)
{
  // One acquisition, one refusal report: perception_node reads these after estimate() returns.
  axis_fit_refusal_reason_.clear();
  axis_fit_refusal_count_ = 0U;
  drop_counts_ = EstimateDropCounts{};
  proposal_diagnostics_.clear();
  if (auto invalid = validate_estimation_inputs(detections, frame, camera_to_planning)) {
    return Result<std::vector<ObservationMessage>>::failure(std::move(*invalid));
  }
  // Re-checked here: estimate() may be handed any well-formed PerceptionFrame, so it cannot assume
  // detect() has validated the buffer it indexes into.
  const auto & depth = frame.depth();
  if (depth.encoding != "32FC1" || depth.step < depth.width * sizeof(float) ||
    depth.step % sizeof(float) != 0U ||
    depth.data.size() < static_cast<std::size_t>(depth.step) * depth.height)
  {
    return failure<std::vector<ObservationMessage>>(
      PerceptionErrorCode::InvalidArgument,
      "depth acquisition is not a 32FC1 image whose buffer covers its declared geometry");
  }

  const auto & intrinsics = frame.intrinsics();
  // validate_estimation_inputs has already proved this transform converts out of the acquisition's
  // own frame, so the point conversion below cannot be applied to points from another frame. The
  // fit and the covariance are both computed in the planning frame, so no value is half-converted.
  const Eigen::Isometry3d planning_from_camera = camera_to_planning.transform();
  const std::string & planning_frame = camera_to_planning.target_frame();

  std::vector<ObservationMessage> observations;
  std::vector<Eigen::Vector3d> points;
  std::vector<double> heights;
  for (const auto & detection : detections.detections) {
    const auto signature = std::ranges::find_if(
      config_.signatures, [&detection](const ProductSignature & candidate) {
        return candidate.product_class == detection.product_class;
      });
    if (signature == config_.signatures.end()) {
      return failure<std::vector<ObservationMessage>>(
        PerceptionErrorCode::NotFound,
        "detection reports a product class this backend has no signature for");
    }

    // Card 065: diagnostics are written to by index; the vector never reallocates under a
    // pointer because nothing else appends to it inside this iteration.
    const EstimateDropCounts drops_before = drop_counts_;
    ProposalDiagnostic * diagnostic = nullptr;
    if (diagnostics_enabled_) {
      proposal_diagnostics_.emplace_back();
      diagnostic = &proposal_diagnostics_.back();
      diagnostic->product_class = detection.product_class;
      diagnostic->x_min = detection.x_min;
      diagnostic->y_min = detection.y_min;
      diagnostic->x_max = detection.x_max;
      diagnostic->y_max = detection.y_max;
    }
    const auto finish = [&]() {
      if (diagnostic != nullptr) {
        diagnostic->verdict = drop_stage(drops_before, drop_counts_);
      }
    };

    points.clear();
    std::size_t mask_pixels = 0;
    const std::size_t box_width = detection.x_max - detection.x_min;
    std::size_t offset = 0;
    bool foreground = false;
    for (const std::uint32_t run : detection.mask.rle_counts) {
      if (foreground) {
        for (std::uint32_t step = 0; step < run; ++step) {
          const std::size_t position = offset + step;
          const std::size_t column = detection.x_min + position % box_width;
          const std::size_t row = detection.y_min + position / box_width;
          ++mask_pixels;
          const double metres =
            sample_depth(depth, row * (depth.step / sizeof(float)) + column);
          if (!std::isfinite(metres) || metres < config_.minimum_depth_m ||
            metres > config_.maximum_depth_m)
          {
            continue;
          }
          const Eigen::Vector3d in_camera(
            (static_cast<double>(column) - intrinsics.cx) * metres / intrinsics.fx,
            (static_cast<double>(row) - intrinsics.cy) * metres / intrinsics.fy, metres);
          const Eigen::Vector3d in_planning = planning_from_camera * in_camera;
          if (config_.workspace.contains(in_planning)) {
            points.push_back(in_planning);
          }
        }
      }
      offset += run;
      foreground = !foreground;
    }
    if (diagnostic != nullptr) {
      diagnostic->mask_pixels = static_cast<std::uint32_t>(mask_pixels);
      diagnostic->points = static_cast<std::uint32_t>(points.size());
      if (!points.empty()) {
        diagnostic->centroid = Eigen::Vector3d::Zero();
        for (const auto & point : points) {
          diagnostic->centroid += point;
        }
        diagnostic->centroid /= static_cast<double>(points.size());
      }
    }
    if (mask_pixels == 0 || points.size() < config_.minimum_top_face_points) {
      ++drop_counts_.too_few_points;
      finish();
      continue;
    }

    heights.clear();
    heights.reserve(points.size());
    for (const auto & point : points) {
      heights.push_back(point.z());
    }
    std::ranges::sort(heights);
    const auto rank = static_cast<std::size_t>(
      config_.top_face_percentile * static_cast<double>(heights.size() - 1));
    const double top_height = heights[rank];
    const double visible_extent = top_height - heights.front();
    if (diagnostic != nullptr) {
      diagnostic->top_height = top_height;
      diagnostic->visible_extent = visible_extent;
    }
    if (visible_extent > signature->nominal_height_m + config_.height_tolerance_m) {
      ++drop_counts_.extent;
      finish();
      continue;
    }

    std::vector<Eigen::Vector3d> band;
    for (const auto & point : points) {
      if (point.z() >= top_height - config_.top_face_band_m) {
        band.push_back(point);
      }
    }
    // How much of what was detected was actually measured in three dimensions inside the cell.
    const double coverage =
      static_cast<double>(points.size()) / static_cast<double>(mask_pixels);

    // Card 063: one colour component can hold several products. Seen obliquely, a same-colour
    // column of a dense tray is one component, because the rear product's barrel fills the gap
    // between the two tops, and a top band holding two discs is not a disc. A duty that enables
    // top_face_split_gap_m has such a band judged disc by disc, all or nothing: every disc must
    // pass every gate, and the admitted centres must stand at least two catalogued radii apart,
    // as two upright cylinders must. Otherwise the whole component is refused. A band with fewer
    // than two discs of usable size takes exactly the unsplit path, and so do tops that touch.
    const auto discs = config_.top_face_split_gap_m > 0.0 ?
      split_into_discs(band, config_.top_face_split_gap_m, config_.minimum_top_face_points) :
      std::vector<std::vector<Eigen::Vector3d>>{};
    std::string * why = nullptr;
    if (diagnostic != nullptr) {
      diagnostic->band_points = static_cast<std::uint32_t>(band.size());
      diagnostic->discs = static_cast<std::uint32_t>(discs.size());
      why = &diagnostic->detail;
    }
    if (discs.size() < 2U) {
      if (auto observation = estimate_disc(
          band, top_height, *signature, detection, detections.header.stamp, planning_frame,
          coverage, why))
      {
        observations.push_back(std::move(*observation));
      }
      finish();
      continue;
    }
    // Card 065 amends Card 063's all-or-nothing rule (Milestone 10 §6, "A column's partly hidden
    // far end does not refuse its front"): every disc is judged; failing discs are set aside only
    // when occlusion explains them (failed_discs_can_be_set_aside). Otherwise the component is
    // refused with exactly the counts the first failing disc alone produced before.
    std::vector<JudgedDisc> judged(discs.size());
    std::vector<std::optional<ObservationMessage>> disc_observations;
    disc_observations.reserve(discs.size());
    std::optional<std::size_t> first_failure;
    EstimateDropCounts counts_at_first_failure;
    std::string reason_at_first_failure;
    std::size_t refusals_at_first_failure = 0U;
    for (std::size_t index = 0; index < discs.size(); ++index) {
      auto observation = estimate_disc(
        discs[index], top_height, *signature, detection, detections.header.stamp,
        planning_frame, coverage, why);
      judged[index].points.reserve(discs[index].size());
      for (const auto & point : discs[index]) {
        judged[index].points.push_back(point.head<2>());
      }
      if (observation.has_value()) {
        judged[index].admitted = true;
        judged[index].fitted_centre = Eigen::Vector2d(
          observation->pose.pose.position.x, observation->pose.pose.position.y);
      } else if (!first_failure) {
        first_failure = index;
        counts_at_first_failure = drop_counts_;
        reason_at_first_failure = axis_fit_refusal_reason_;
        refusals_at_first_failure = axis_fit_refusal_count_;
      }
      disc_observations.push_back(std::move(observation));
    }
    std::vector<ObservationMessage> split;
    SetAsideDecision decision;
    decision.answer = SetAsideAnswer::kNothingFailed;
    if (first_failure) {
      decision = judge_failed_discs(
        judged, planning_from_camera.translation().head<2>(), signature->nominal_radius_m);
    }
    const bool set_aside = decision.set_aside();
    if (diagnostic != nullptr && first_failure) {
      diagnostic->set_aside_answer = set_aside_answer_name(decision.answer);
      diagnostic->minimum_clearance_m = decision.minimum_clearance_m;
    }
    if (!first_failure || set_aside) {
      for (auto & observation : disc_observations) {
        if (observation.has_value()) {
          split.push_back(std::move(*observation));
        }
      }
      if (set_aside && why != nullptr) {
        char clearance[48];
        std::snprintf(
          clearance, sizeof(clearance), " set_aside clearance=%.4f", decision.minimum_clearance_m);
        *why += clearance;
      }
    } else {
      drop_counts_ = counts_at_first_failure;
      axis_fit_refusal_reason_ = reason_at_first_failure;
      axis_fit_refusal_count_ = refusals_at_first_failure;
      if (why != nullptr) {
        char refused[80];
        std::snprintf(
          refused, sizeof(refused), " refused: %s clearance=%.4f",
          set_aside_answer_name(decision.answer), decision.minimum_clearance_m);
        *why += refused;
      }
    }
    for (std::size_t first = 0; first < split.size(); ++first) {
      for (std::size_t second = first + 1U; second < split.size(); ++second) {
        const double separation = std::hypot(
          split[first].pose.pose.position.x - split[second].pose.pose.position.x,
          split[first].pose.pose.position.y - split[second].pose.pose.position.y);
        if (separation < 2.0 * signature->nominal_radius_m) {
          ++drop_counts_.split_overlap;
          split.clear();
        }
      }
    }
    const bool published = !split.empty();
    for (auto & observation : split) {
      observations.push_back(std::move(observation));
    }
    finish();
    if (diagnostic != nullptr && set_aside && published) {
      // The set-aside discs moved a counter, but the component itself was published.
      diagnostic->verdict = "published (set_aside)";
    }
  }
  return Result<std::vector<ObservationMessage>>::success(std::move(observations));
}


std::optional<ObservationMessage> ColourDepthBackend::estimate_disc(
  const std::vector<Eigen::Vector3d> & top_face, double top_height,
  const ProductSignature & signature, const DetectionMessage & detection,
  const builtin_interfaces::msg::Time & stamp, const std::string & planning_frame,
  double coverage, std::string * why)
{
  // Card 065, diagnostic only: one "[...]" entry per judged disc, appended when asked for.
  const auto note = [why, &top_face](const std::string & text) {
    if (why == nullptr) {
      return;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto & point : top_face) {
      mean += point;
    }
    if (!top_face.empty()) {
      mean /= static_cast<double>(top_face.size());
    }
    char head[96];
    std::snprintf(
      head, sizeof(head), "[(%.3f,%.3f) n=%zu ", mean.x(), mean.y(), top_face.size());
    *why += head + text + "]";
  };
  if (top_face.size() < config_.minimum_top_face_points) {
    ++drop_counts_.top_face_points;
    note("top_face_points");
    return std::nullopt;
  }
  Eigen::Vector2d rough_centroid = Eigen::Vector2d::Zero();
  for (const auto & point : top_face) {
    rough_centroid += point.head<2>();
  }
  rough_centroid /= static_cast<double>(top_face.size());

  // The disc the camera sees of an upright cylinder's top has a known radial profile. This gate
  // separates a product from anything else of its colour: the robot's own links classify as
  // small-bottle blue and fail here by a factor of four. It runs *before* the axis-fit quality
  // gate below so the common non-product blobs never reach the fit (and never appear in the
  // node's refusal log); both are independent continue filters, so the published set is
  // unchanged either way.
  //
  // The gate is measured about the band's *mean*, not the fitted axis: the shipped radius-ratio
  // bands were re-derived that way (wrist_tray_perception.yaml), and measuring them about the
  // circle fit would widen the set of tilts the geometry stage admits (test_non_upright_
  // container's disjoint bands). Only the published position uses the fitted axis below.
  double mean_radius = 0.0;
  double extreme_radius = 0.0;
  double height_variance = 0.0;
  for (const auto & point : top_face) {
    const double radius = (point.head<2>() - rough_centroid).norm();
    mean_radius += radius;
    extreme_radius = std::max(extreme_radius, radius);
    height_variance += (point.z() - top_height) * (point.z() - top_height);
  }
  const auto sample_count = static_cast<double>(top_face.size());
  mean_radius /= sample_count;
  height_variance /= sample_count;
  const double mean_ratio = mean_radius / signature.nominal_radius_m;
  const double extreme_ratio = extreme_radius / signature.nominal_radius_m;
  if (mean_ratio < config_.minimum_mean_radius_ratio ||
    mean_ratio > config_.maximum_mean_radius_ratio ||
    extreme_ratio < config_.minimum_extreme_radius_ratio ||
    extreme_ratio > config_.maximum_extreme_radius_ratio)
  {
    ++drop_counts_.radial_profile;
    char ratios[80];
    std::snprintf(
      ratios, sizeof(ratios), "radial_profile mean=%.2f extreme=%.2f", mean_ratio,
      extreme_ratio);
    note(ratios);
    return std::nullopt;
  }

  // Axis from the band's silhouette, not the band's mean.
  //
  // At the shipped 51-degree tray elevation the world-Z band that selects the top face also
  // admits a strip of the camera-facing barrel, and the deprojected disc is sampled far more
  // densely on the near rim. A plain XY mean is therefore pulled toward the camera — a
  // one-signed world-Y offset of 2 to 7 mm on the catalogued products (Card 004
  // residual_xyz_m; root-caused on Card 036). Every hull vertex of the band's XY points lies
  // on the rim or the lateral surface, and both share the same world-XY circle about the
  // upright axis, so an algebraic circle fit to the hull recovers the axis regardless of
  // barrel fraction and density skew.
  //
  // The rim-circle assumption is not taken on trust: axis_fit_refusal_reason() rejects a fit
  // whose hull is a short arc, rank-deficient, off-centre, or not a circle (occluder chord,
  // merged same-colour products that somehow cleared the radius gate) and the detection is
  // dropped without a pose — fail-closed on position (Card 036 review follow-up 1). The
  // backend has no node handle, so the reason is stashed for perception_node to emit through
  // its throttled logger.
  std::vector<Eigen::Vector2d> band_xy;
  band_xy.reserve(top_face.size());
  for (const auto & point : top_face) {
    band_xy.push_back(point.head<2>());
  }
  const std::vector<Eigen::Vector2d> hull = axis_fit::convex_hull_2d(band_xy);
  const axis_fit::AxisFit fit = axis_fit::fit_circle_axis(hull, rough_centroid);
  const char * refusal = axis_fit::refusal_reason(fit, rough_centroid, extreme_radius);
  if (refusal != nullptr) {
    if (axis_fit_refusal_reason_.empty()) {
      axis_fit_refusal_reason_ = refusal;
    }
    ++axis_fit_refusal_count_;
    ++drop_counts_.axis_fit;
    note(std::string("axis_fit: ") + refusal);
    return std::nullopt;
  }
  const Eigen::Vector2d & centroid = fit.centre;
  if (!centroid.allFinite()) {
    ++drop_counts_.nonfinite_centre;
    note("nonfinite_centre");
    return std::nullopt;
  }
  note("ok");

  // A horizontal disc of the catalogued radius, that high above the lowest visible point of a
  // blob no taller than the product, suggests a cylinder standing on its base. This does not
  // verify uprightness; see the rotational covariance below.
  Eigen::Vector2d scatter = Eigen::Vector2d::Zero();
  for (const auto & point : top_face) {
    scatter += (point.head<2>() - rough_centroid).cwiseAbs2();
  }
  scatter /= sample_count;
  const double floor_variance =
    config_.translation_sigma_floor_m * config_.translation_sigma_floor_m;

  ObservationMessage observation;
  // The acquisition instant, never now(): the world state's staleness gate measures the age of
  // the measurement, and stamping at publication would disable it.
  observation.header.stamp = stamp;
  observation.header.frame_id = planning_frame;
  // Left empty: this backend measures where a product is, not which product it is. Identity is
  // assigned downstream, and an empty source identity is rejected by the world state's contract
  // validation rather than becoming a per-frame identity that churns.
  observation.source_object_id.clear();
  observation.product_class = detection.product_class;
  observation.has_sku = true;
  observation.sku = signature.sku;
  // An assumption, not a classification. It is written unconditionally (no path here produces
  // ORIENTATION_HORIZONTAL, ORIENTATION_TILTED or ORIENTATION_UNKNOWN), and
  // test_non_upright_container measures the cost: a can tilted 51 degrees clears every geometry
  // gate and arrives downstream labelled upright. The covariance below declares this. Changing
  // the label is a downstream decision, because eligible_object refuses anything that is not
  // ORIENTATION_UPRIGHT and would refuse every product in the cell.
  observation.orientation = ObservationMessage::ORIENTATION_UPRIGHT;
  observation.pose.pose.position.x = centroid.x();
  observation.pose.pose.position.y = centroid.y();
  observation.pose.pose.position.z = top_height - signature.nominal_height_m / 2.0;
  observation.pose.pose.orientation.w = 1.0;

  for (double & entry : observation.pose.covariance) {
    entry = 0.0;
  }
  // x and y are the axis of a circle fit to the band's convex hull; their uncertainty is the
  // standard error of the band's samples about that axis and shrinks with the sample count.
  // z comes from an order statistic of the same points, whose uncertainty is the scatter of
  // those points and does not shrink by averaging. The accuracy floor below is what makes the
  // claim cover measured error against ground truth (Milestone 10 §7).
  observation.pose.covariance[0] = floor_variance + scatter.x() / sample_count;
  observation.pose.covariance[7] = floor_variance + scatter.y() / sample_count;
  observation.pose.covariance[14] = floor_variance + height_variance;
  // All three rotational degrees of freedom carry the variance of a uniform distribution over a
  // full turn, which is this package's way of saying the quantity was not measured. The
  // quaternion above is a constant, and the covariance is the only thing that tells a consumer
  // so: an identity quaternion from an estimator that fitted an orientation and one from an
  // estimator that never looked are the same six numbers.
  //
  // Yaw is unobservable on a surface of revolution. Roll and pitch are unmeasured because the
  // geometry gates do not verify uprightness: test_non_upright_container shows that at the
  // shipped thresholds a can tilted 51 degrees off vertical passes every gate and is published
  // with this identity quaternion. A sigma of 0.10 rad against an error of 0.89 rad would have
  // been a false claim.
  observation.pose.covariance[21] = kUnmeasuredRotationVariance;
  observation.pose.covariance[28] = kUnmeasuredRotationVariance;
  observation.pose.covariance[35] = kUnmeasuredRotationVariance;

  // How much of what was detected was actually measured in three dimensions inside the cell.
  // The chain is no more confident than its weakest stage.
  observation.confidence =
    static_cast<float>(std::min(static_cast<double>(detection.score), coverage));
  observation.backend_name = config_.backend_name;
  observation.backend_version = config_.backend_version;
  observation.status = ObservationMessage::STATUS_OK;
  return observation;
}

}  // namespace restocker_perception
