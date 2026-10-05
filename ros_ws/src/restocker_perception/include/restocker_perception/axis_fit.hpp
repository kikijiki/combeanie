// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Core>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace restocker_perception
{
namespace axis_fit
{

// What an algebraic circle fit produced, and the numbers the acceptance gate reads.
struct AxisFit
{
  Eigen::Vector2d centre{Eigen::Vector2d::Zero()};
  Eigen::Index rank{0};
  bool solved{false};
  // Angular span of the hull about the rough centroid, in [0, 2*pi].
  double span_rad{0.0};
  // Mean |p - centre| over the hull — the fitted radius, in metres.
  double mean_radius{0.0};
  // RMS of |p - centre| minus the fitted radius over the hull: 0 for points on a circle.
  double geometric_rms_residual{0.0};
};

// Andrew's monotone chain. Returns the hull in counter-clockwise order without repeating the
// first vertex.
[[nodiscard]] inline std::vector<Eigen::Vector2d> convex_hull_2d(
  std::vector<Eigen::Vector2d> points)
{
  const auto n = points.size();
  if (n < 3U) {
    return points;
  }
  std::ranges::sort(
    points, [](const Eigen::Vector2d & left, const Eigen::Vector2d & right) {
      return left.x() < right.x() || (left.x() == right.x() && left.y() < right.y());
    });
  const auto cross = [](const Eigen::Vector2d & o, const Eigen::Vector2d & a,
    const Eigen::Vector2d & b) {
    return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
  };
  std::vector<Eigen::Vector2d> hull;
  hull.reserve(2U * n);
  for (const auto & point : points) {
    while (hull.size() >= 2U &&
      cross(hull[hull.size() - 2U], hull.back(), point) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const auto lower_size = hull.size();
  for (std::size_t index = n - 1U; index-- > 0U; ) {
    const auto & point = points[index];
    while (hull.size() > lower_size &&
      cross(hull[hull.size() - 2U], hull.back(), point) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  hull.pop_back();
  return hull;
}

// Algebraic (Kåsa) circle fit: least squares of x² + y² = 2·cx·x + 2·cy·y + k, plus the
// diagnostics the acceptance gate needs.
//
// Assumption the fit rests on: **every hull vertex lies on the product's rim circle** — true for
// an upright axisymmetric product's rim and lateral surface (both are the same world-XY circle
// about the axis), false when an occluder's straight cut crosses the disc interior or when two
// same-colour products merge into one component. axis_fit::refusal_reason() below is what turns
// a violated assumption into a refusal instead of a silently wrong centre.
[[nodiscard]] inline AxisFit fit_circle_axis(
  const std::vector<Eigen::Vector2d> & hull, const Eigen::Vector2d & rough_centroid)
{
  AxisFit fit;
  if (hull.size() < 3U) {
    return fit;
  }
  // Angular span about the rough centroid: sort the polar angles and take 2*pi minus the
  // largest gap, so a partial arc reports its own span rather than a naive max-min.
  std::vector<double> angles;
  angles.reserve(hull.size());
  for (const auto & point : hull) {
    angles.push_back(
      std::atan2(point.y() - rough_centroid.y(), point.x() - rough_centroid.x()));
  }
  std::ranges::sort(angles);
  double largest_gap = angles.front() + 2.0 * M_PI - angles.back();
  for (std::size_t index = 1; index < angles.size(); ++index) {
    largest_gap = std::max(largest_gap, angles[index] - angles[index - 1U]);
  }
  fit.span_rad = 2.0 * M_PI - largest_gap;

  Eigen::MatrixXd design(static_cast<Eigen::Index>(hull.size()), 3);
  Eigen::VectorXd rhs(static_cast<Eigen::Index>(hull.size()));
  for (std::size_t index = 0; index < hull.size(); ++index) {
    const auto row = static_cast<Eigen::Index>(index);
    design(row, 0) = 2.0 * hull[index].x();
    design(row, 1) = 2.0 * hull[index].y();
    design(row, 2) = 1.0;
    rhs(row) = hull[index].squaredNorm();
  }
  const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
  fit.rank = qr.rank();
  if (fit.rank < 3) {
    return fit;
  }
  const Eigen::Vector3d solution = qr.solve(rhs);
  if (!solution.allFinite()) {
    return fit;
  }
  fit.centre = solution.head<2>();
  double radius_sum = 0.0;
  for (const auto & point : hull) {
    radius_sum += (point - fit.centre).norm();
  }
  fit.mean_radius = radius_sum / static_cast<double>(hull.size());
  double residual_sum = 0.0;
  for (const auto & point : hull) {
    const double residual = (point - fit.centre).norm() - fit.mean_radius;
    residual_sum += residual * residual;
  }
  fit.geometric_rms_residual = std::sqrt(residual_sum / static_cast<double>(hull.size()));
  fit.solved = true;
  return fit;
}

// Fit-quality gate (Card 036 review follow-up 1). Returns nullptr when the fit may be published,
// otherwise the refusal reason: the detection is then dropped without a pose — fail-closed on
// position, because a centre a radius away from where the product actually is is a
// safety-relevant perception failure, not a degraded measurement. The strings are what
// perception_node emits through its throttled logger and what the tests pin.
//
// Every hull vertex must lie on the product's rim circle (see fit_circle_axis). The checks below
// are the observable consequences of that assumption: a short arc or a rank-deficient design fits
// a circle to machine precision with a centre about one radius off; a chord-cut or merged
// silhouette leaves interior points on the hull and blows the geometric residual; a fit that
// wanders further from the band mean than the band's own extreme radius cannot be an axis
// inside that band.
[[nodiscard]] inline const char * refusal_reason(
  const AxisFit & fit, const Eigen::Vector2d & rough_centroid, double extreme_radius)
{
  if (!fit.solved || fit.rank < 3) {
    return "circle-fit design is degenerate (fewer than three hull vertices, or rank-deficient)";
  }
  if (fit.span_rad < M_PI) {
    return
      "band silhouette spans less than half a turn about the band mean (short arc or chord cut)";
  }
  if ((fit.centre - rough_centroid).norm() > extreme_radius) {
    return "fitted axis lies further from the band mean than the band's extreme radius";
  }
  if (fit.geometric_rms_residual > 0.15 * fit.mean_radius) {
    // A healthy fit on exact depth has a sub-micrometre residual; 15% of the fitted radius is
    // far above that (and above the ~1 mm hull residual the review measured at 1 mm depth
    // noise) but far below a peanut-shaped merged component.
    return "algebraic circle fit has a large geometric residual (hull is not a circle)";
  }
  return nullptr;
}

}  // namespace axis_fit
}  // namespace restocker_perception
