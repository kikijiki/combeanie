// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// The lane depletion measurement, against a depth image this file renders itself.
//
// A rendered scene rather than a hand-written buffer, because the properties worth asserting are
// geometric: what the bed does to the floor test, what a cylinder's rear tangent does to the
// nearest-surface search, and what a product's own shadow does to coverage when the product is
// invisible. None of those survives a buffer with three numbers in it.
//
// The camera pose is the shipped lane station and the lane window is the shipped workcell, so a
// change to either that breaks the measurement breaks this test rather than the simulator.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "restocker_perception/lane_depth_measurement.hpp"
#include "restocker_perception/survey_stations.hpp"

namespace
{

using restocker_perception::DepthCameraIntrinsics;
using restocker_perception::LaneDepthConfig;
using restocker_perception::LaneDepthWindow;
using restocker_perception::lane_depth_window;
using restocker_perception::measure_lane_depth;
using restocker_perception::WorkcellSurveyGeometry;

// The shipped wrist camera, from restocker_description/urdf/sensors.xacro.
constexpr std::uint32_t kWidth = 1280U;
constexpr std::uint32_t kHeight = 720U;
constexpr double kHorizontalFov = 1.48;

// The measurement's own defaults, restated so a test failure names the value it was run with.
constexpr double kFloorMarginM = 0.010;
constexpr double kFrontInsetM = 0.010;
constexpr double kObstructionReachM = 0.060;

[[nodiscard]] std::string workcell_geometry_path()
{
  const char * path = std::getenv("RESTOCKER_TEST_WORKCELL_GEOMETRY");
  return path == nullptr ? std::string() : std::string(path);
}

[[nodiscard]] DepthCameraIntrinsics wrist_intrinsics()
{
  DepthCameraIntrinsics intrinsics;
  intrinsics.width = kWidth;
  intrinsics.height = kHeight;
  intrinsics.fx = (static_cast<double>(kWidth) / 2.0) / std::tan(kHorizontalFov / 2.0);
  intrinsics.fy = intrinsics.fx;
  intrinsics.cx = (static_cast<double>(kWidth) - 1.0) / 2.0;
  intrinsics.cy = (static_cast<double>(kHeight) - 1.0) / 2.0;
  return intrinsics;
}

// An upright cylinder standing on the bed, in the lane's own frame.
struct Product
{
  double center_depth_m{0.0};
  double radius_m{0.0};
  double height_m{0.0};
  // A product the depth stream does not return. This is the failure the coverage rule exists to
  // catch, and modelling it is the only way to assert that it is caught.
  bool invisible{false};
};

// The bed, the retainer, the dividers and whatever products are standing in the lane.
//
// Ray casting rather than rasterisation: every primitive here has a closed-form intersection, and
// the depth value a pinhole camera writes is the Z coordinate in its own optical frame, so a ray
// parameterised by the unnormalised direction ((u - cx)/fx, (v - cy)/fy, 1) has its parameter
// equal to that depth. Nothing here has to normalise anything.
class LaneScene
{
public:
  LaneScene(const LaneDepthWindow & window, double lane_half_span_m, double divider_height_m)
  : window_(window), lane_half_span_m_(lane_half_span_m), divider_height_m_(divider_height_m) {}

  void add(const Product & product) {products_.push_back(product);}

  [[nodiscard]] std::optional<double> cast(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & direction) const
  {
    std::optional<double> nearest;
    const auto keep = [&nearest](std::optional<double> candidate) {
      if (candidate && *candidate > 0.0 && (!nearest || *candidate < *nearest)) {
        nearest = candidate;
      }
    };
    keep(bed(origin, direction));
    keep(retainer(origin, direction));
    keep(divider(origin, direction, lane_half_span_m_));
    keep(divider(origin, direction, -lane_half_span_m_));
    for (const Product & product : products_) {
      if (!product.invisible) {
        keep(cylinder(origin, direction, product));
      }
    }
    return nearest;
  }

private:
  // z = (datum - y) * slope, bounded to the shelf's own footprint.
  [[nodiscard]] std::optional<double> bed(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & direction) const
  {
    const Eigen::Vector3d normal(0.0, window_.bed_slope, 1.0);
    const double offset = window_.bed_datum_depth_m * window_.bed_slope;
    const double denominator = normal.dot(direction);
    if (std::abs(denominator) < 1.0e-12) {
      return std::nullopt;
    }
    const double t = (offset - normal.dot(origin)) / denominator;
    const Eigen::Vector3d hit = origin + t * direction;
    if (std::abs(hit.x()) > lane_half_span_m_ || hit.y() < -0.02 ||
      hit.y() > window_.bed_datum_depth_m + 0.06)
    {
      return std::nullopt;
    }
    return t;
  }

  [[nodiscard]] std::optional<double> retainer(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & direction) const
  {
    // The rear face only: nothing in the lane can see the other three.
    if (std::abs(direction.y()) < 1.0e-12) {
      return std::nullopt;
    }
    const double t = (window_.bed_datum_depth_m - origin.y()) / direction.y();
    const Eigen::Vector3d hit = origin + t * direction;
    if (std::abs(hit.x()) > lane_half_span_m_ || hit.z() < 0.0 || hit.z() > kRetainerHeightM) {
      return std::nullopt;
    }
    return t;
  }

  [[nodiscard]] std::optional<double> divider(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & direction, double plane_x) const
  {
    if (std::abs(direction.x()) < 1.0e-12) {
      return std::nullopt;
    }
    const double t = (plane_x - origin.x()) / direction.x();
    const Eigen::Vector3d hit = origin + t * direction;
    const double bed_height = window_.bed_height_at(hit.y());
    if (hit.y() < 0.0 || hit.y() > window_.bed_datum_depth_m + 0.06 || hit.z() < bed_height ||
      hit.z() > bed_height + divider_height_m_)
    {
      return std::nullopt;
    }
    return t;
  }

  [[nodiscard]] std::optional<double> cylinder(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & direction,
    const Product & product) const
  {
    const double base = window_.bed_height_at(product.center_depth_m);
    const double top = base + product.height_m;
    std::optional<double> nearest;
    const double a =
      (direction.x() * direction.x()) + (direction.y() * direction.y());
    if (a > 1.0e-14) {
      const double dx = origin.x();
      const double dy = origin.y() - product.center_depth_m;
      const double b = 2.0 * ((direction.x() * dx) + (direction.y() * dy));
      const double c = (dx * dx) + (dy * dy) - (product.radius_m * product.radius_m);
      const double discriminant = (b * b) - (4.0 * a * c);
      if (discriminant >= 0.0) {
        const double root = std::sqrt(discriminant);
        for (const double t : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)}) {
          const Eigen::Vector3d hit = origin + t * direction;
          if (t > 0.0 && hit.z() >= base && hit.z() <= top && (!nearest || t < *nearest)) {
            nearest = t;
          }
        }
      }
    }
    if (std::abs(direction.z()) > 1.0e-12) {
      const double t = (top - origin.z()) / direction.z();
      const Eigen::Vector3d hit = origin + t * direction;
      const double dx = hit.x();
      const double dy = hit.y() - product.center_depth_m;
      if (t > 0.0 && (dx * dx) + (dy * dy) <= product.radius_m * product.radius_m &&
        (!nearest || t < *nearest))
      {
        nearest = t;
      }
    }
    return nearest;
  }

  static constexpr double kRetainerHeightM = 0.09;

  LaneDepthWindow window_;
  double lane_half_span_m_{0.0};
  double divider_height_m_{0.0};
  std::vector<Product> products_;
};

[[nodiscard]] std::vector<float> render(
  const LaneScene & scene, const DepthCameraIntrinsics & intrinsics,
  const Eigen::Isometry3d & lane_from_optical)
{
  std::vector<float> depth(
    static_cast<std::size_t>(intrinsics.width) * intrinsics.height,
    std::numeric_limits<float>::quiet_NaN());
  const Eigen::Vector3d origin = lane_from_optical.translation();
  for (std::uint32_t row = 0U; row < intrinsics.height; ++row) {
    for (std::uint32_t column = 0U; column < intrinsics.width; ++column) {
      const Eigen::Vector3d in_optical(
        (static_cast<double>(column) - intrinsics.cx) / intrinsics.fx,
        (static_cast<double>(row) - intrinsics.cy) / intrinsics.fy, 1.0);
      const auto hit = scene.cast(origin, lane_from_optical.linear() * in_optical);
      if (hit) {
        depth[(static_cast<std::size_t>(row) * intrinsics.width) + column] =
          static_cast<float>(*hit);
      }
    }
  }
  return depth;
}

// The whole fixture the tests share: the shipped workcell, one lane's window, and the camera pose
// the shipped station table puts the eye at for that lane.
struct LaneFixture
{
  WorkcellSurveyGeometry geometry;
  LaneDepthWindow window;
  Eigen::Isometry3d lane_from_optical{Eigen::Isometry3d::Identity()};
  double lane_half_span_m{0.0};
  double divider_height_m{0.0};
};

[[nodiscard]] std::optional<LaneFixture> shipped_lane(const std::string & lane_id)
{
  const std::string path = workcell_geometry_path();
  if (path.empty()) {
    return std::nullopt;
  }
  LaneFixture fixture;
  fixture.geometry = restocker_perception::load_workcell_survey_geometry(path);
  const restocker_perception::LaneSurveyGeometry * lane =
    restocker_perception::find_lane_geometry(fixture.geometry, lane_id);
  if (lane == nullptr) {
    return std::nullopt;
  }
  auto window = lane_depth_window(
    *lane, fixture.geometry.shelf, kFloorMarginM, kFrontInsetM, kObstructionReachM);
  if (!window) {
    return std::nullopt;
  }
  fixture.window = window.value();
  // The clear span between consecutive dividers, which is the usable width plus one side
  // clearance at each side. The scene needs the divider faces, not the inset band.
  fixture.lane_half_span_m = (lane->usable_width_m / 2.0) + lane->side_clearance_m;
  fixture.divider_height_m = 0.42;

  const std::vector<restocker_perception::SurveyStation> stations =
    restocker_perception::nominal_survey_stations(fixture.geometry);
  const restocker_perception::SurveyStation * station =
    restocker_perception::find_survey_station(stations, lane_id);
  if (station == nullptr) {
    return std::nullopt;
  }
  // A lane frame is the shelf frame translated along X by the lane's centre, and nothing else.
  Eigen::Isometry3d lane_from_shelf = Eigen::Isometry3d::Identity();
  lane_from_shelf.translation().x() = -lane->center_x_m;
  fixture.lane_from_optical = lane_from_shelf * station->shelf_from_optical;
  return fixture;
}

// Where the ground-truth producer would put the rear face of an upright cylinder: the same
// axis-aligned bound restocker_gazebo/lane_evidence.cpp takes, so the two agree by construction
// and any disagreement this test reports belongs to the camera measurement.
[[nodiscard]] double ground_truth_available_depth(
  const LaneDepthWindow & window, const std::vector<Product> & products)
{
  double available = window.usable_depth_m;
  for (const Product & product : products) {
    available = std::min(
      available,
      std::clamp(
        product.center_depth_m - product.radius_m - window.rear_clearance_m, 0.0,
        window.usable_depth_m));
  }
  return available;
}

// A settled column of `count` products packed forward against the retainer, front first.
[[nodiscard]] std::vector<Product> settled_column(
  const LaneDepthWindow & window, std::size_t count, double radius_m, double height_m)
{
  std::vector<Product> products;
  for (std::size_t index = 0U; index < count; ++index) {
    Product product;
    product.radius_m = radius_m;
    product.height_m = height_m;
    product.center_depth_m =
      window.bed_datum_depth_m - radius_m - (2.0 * radius_m * static_cast<double>(index));
    products.push_back(product);
  }
  return products;
}

TEST(LaneDepthMeasurement, SeesAnEmptyLaneAlongItsWholeLength) {
  const auto fixture = shipped_lane("lane_03");
  ASSERT_TRUE(fixture.has_value()) << "RESTOCKER_TEST_WORKCELL_GEOMETRY is not set";
  const DepthCameraIntrinsics intrinsics = wrist_intrinsics();
  const LaneScene scene(
    fixture->window, fixture->lane_half_span_m, fixture->divider_height_m);
  const std::vector<float> depth = render(scene, intrinsics, fixture->lane_from_optical);

  const auto measured = measure_lane_depth(
    depth, intrinsics.width, intrinsics, fixture->lane_from_optical, fixture->window,
    LaneDepthConfig{});
  ASSERT_TRUE(measured.has_value()) << measured.error().detail;
  EXPECT_FALSE(measured.value().nearest_surface_found);
  EXPECT_DOUBLE_EQ(measured.value().available_depth_m, fixture->window.usable_depth_m);
  // The whole point of the inclined floor test. With a floor of constant height the bed itself
  // would be the nearest surface over the rear two thirds of the lane and this would read as
  // full; with a floor that follows the bed, an empty lane is bed all the way down and coverage
  // is complete.
  EXPECT_GT(measured.value().coverage, 0.99);
  EXPECT_EQ(measured.value().product_returns, 0U);
  EXPECT_GT(measured.value().bed_returns, 1000U);
}

TEST(LaneDepthMeasurement, AgreesWithGroundTruthOverTheWholeColumnRange) {
  const auto fixture = shipped_lane("lane_03");
  ASSERT_TRUE(fixture.has_value()) << "RESTOCKER_TEST_WORKCELL_GEOMETRY is not set";
  const DepthCameraIntrinsics intrinsics = wrist_intrinsics();

  struct Catalogued
  {
    const char * name;
    double radius_m;
    double height_m;
    std::size_t maximum;
  };
  // The shipped catalogue, and the per-class capacity the handover records.
  const std::vector<Catalogued> catalogue{
    {"can", 0.033, 0.122, 12U},
    {"small_bottle", 0.034, 0.200, 11U},
    {"large_bottle", 0.045, 0.290, 8U}};

  double worst_absolute = 0.0;
  double worst_positive = 0.0;
  for (const Catalogued & entry : catalogue) {
    for (std::size_t count = 0U; count <= entry.maximum; ++count) {
      const std::vector<Product> products =
        settled_column(fixture->window, count, entry.radius_m, entry.height_m);
      LaneScene scene(fixture->window, fixture->lane_half_span_m, fixture->divider_height_m);
      for (const Product & product : products) {
        scene.add(product);
      }
      const std::vector<float> depth = render(scene, intrinsics, fixture->lane_from_optical);
      const auto measured = measure_lane_depth(
        depth, intrinsics.width, intrinsics, fixture->lane_from_optical, fixture->window,
        LaneDepthConfig{});
      ASSERT_TRUE(measured.has_value()) << measured.error().detail;
      const double truth = ground_truth_available_depth(fixture->window, products);
      const double error = measured.value().available_depth_m - truth;
      worst_absolute = std::max(worst_absolute, std::abs(error));
      worst_positive = std::max(worst_positive, error);
      std::cout << entry.name << " x" << count << ": measured "
                << measured.value().available_depth_m << " truth " << truth << " error "
                << (error * 1000.0) << " mm, coverage " << measured.value().coverage << "\n";
      EXPECT_GT(measured.value().coverage, 0.95)
        << entry.name << " x" << count << " was not seen along its whole visible length";
      // 10 mm is the assertable bound this rendering supports and is an order of magnitude below
      // the smallest product pitch. The measured worst case is printed above on every run.
      EXPECT_LT(std::abs(error), 0.010) << entry.name << " x" << count;
    }
  }
  std::cout << "worst |error| " << (worst_absolute * 1000.0) << " mm, worst emptier-than-truth "
            << (worst_positive * 1000.0) << " mm\n";
}

TEST(LaneDepthMeasurement, ABlindedCameraIsNotAnEmptyLane) {
  const auto fixture = shipped_lane("lane_03");
  ASSERT_TRUE(fixture.has_value()) << "RESTOCKER_TEST_WORKCELL_GEOMETRY is not set";
  const DepthCameraIntrinsics intrinsics = wrist_intrinsics();
  const std::size_t pixels =
    static_cast<std::size_t>(intrinsics.width) * intrinsics.height;

  for (const float blank :
    {std::numeric_limits<float>::quiet_NaN(), 0.0F, -1.0F,
      std::numeric_limits<float>::infinity()})
  {
    const std::vector<float> depth(pixels, blank);
    const auto measured = measure_lane_depth(
      depth, intrinsics.width, intrinsics, fixture->lane_from_optical, fixture->window,
      LaneDepthConfig{});
    ASSERT_TRUE(measured.has_value()) << measured.error().detail;
    // The depth value it reports is the emptiest one there is, exactly as section 3 of the
    // specification says it must be. What stops that being believed is that nothing whatever
    // backs it: no bin was accounted for, so coverage is zero and a consumer gating on coverage
    // never sees the number.
    EXPECT_DOUBLE_EQ(measured.value().available_depth_m, fixture->window.usable_depth_m);
    EXPECT_DOUBLE_EQ(measured.value().coverage, 0.0);
    EXPECT_EQ(measured.value().covered_bins, 0U);
    EXPECT_EQ(measured.value().valid_depth_samples, 0U);
  }
}

TEST(LaneDepthMeasurement, ACameraBlindedInPartLosesTheCoverageItDidNotEarn) {
  const auto fixture = shipped_lane("lane_03");
  ASSERT_TRUE(fixture.has_value()) << "RESTOCKER_TEST_WORKCELL_GEOMETRY is not set";
  const DepthCameraIntrinsics intrinsics = wrist_intrinsics();
  const LaneScene scene(fixture->window, fixture->lane_half_span_m, fixture->divider_height_m);
  std::vector<float> depth = render(scene, intrinsics, fixture->lane_from_optical);

  // The upper half of the image looks down the far half of the lane. Blanking it is what a lens
  // fouled at one edge, or a shroud, or a partial renderer failure looks like: the depth stream
  // is alive, its stamps advance, and half the lane is not there.
  for (std::uint32_t row = 0U; row < intrinsics.height / 2U; ++row) {
    for (std::uint32_t column = 0U; column < intrinsics.width; ++column) {
      depth[(static_cast<std::size_t>(row) * intrinsics.width) + column] =
        std::numeric_limits<float>::quiet_NaN();
    }
  }
  const auto measured = measure_lane_depth(
    depth, intrinsics.width, intrinsics, fixture->lane_from_optical, fixture->window,
    LaneDepthConfig{});
  ASSERT_TRUE(measured.has_value()) << measured.error().detail;
  // It still reports an empty lane, because in the half it can see the lane is empty.
  EXPECT_DOUBLE_EQ(measured.value().available_depth_m, fixture->window.usable_depth_m);
  // And it still cannot be believed, because it did not see the other half.
  EXPECT_LT(measured.value().coverage, 0.90);
  EXPECT_GT(measured.value().coverage, 0.0);
  std::cout << "half a blinded frame on an empty lane: coverage " << measured.value().coverage
            << "\n";
}

TEST(LaneDepthMeasurement, ACameraAimedAtTheNeighbouringLaneEarnsNoCoverageHere) {
  const auto fixture = shipped_lane("lane_03");
  const auto neighbour = shipped_lane("lane_04");
  ASSERT_TRUE(fixture.has_value()) << "RESTOCKER_TEST_WORKCELL_GEOMETRY is not set";
  ASSERT_TRUE(neighbour.has_value());
  const DepthCameraIntrinsics intrinsics = wrist_intrinsics();

  // lane_04 is full, lane_03 is empty, and the camera is at lane_04's station. Asking about
  // lane_03 from there must not answer "empty" on the strength of having seen a different lane.
  LaneScene scene(neighbour->window, neighbour->lane_half_span_m, neighbour->divider_height_m);
  for (const Product & product : settled_column(neighbour->window, 12U, 0.033, 0.122)) {
    scene.add(product);
  }
  const std::vector<float> depth = render(scene, intrinsics, neighbour->lane_from_optical);
  // The same camera pose, re-expressed in lane_03's frame: the lanes are 0.4 m apart in X.
  Eigen::Isometry3d lane_03_from_lane_04 = Eigen::Isometry3d::Identity();
  lane_03_from_lane_04.translation().x() = 0.4;
  const auto measured = measure_lane_depth(
    depth, intrinsics.width, intrinsics, lane_03_from_lane_04 * neighbour->lane_from_optical,
    fixture->window, LaneDepthConfig{});
  ASSERT_TRUE(measured.has_value()) << measured.error().detail;
  EXPECT_DOUBLE_EQ(measured.value().coverage, 0.0);
  EXPECT_EQ(measured.value().product_returns, 0U);
}

TEST(LaneDepthMeasurement, CannotSeeAProductTheDepthStreamPassesThrough) {
  const auto fixture = shipped_lane("lane_03");
  ASSERT_TRUE(fixture.has_value()) << "RESTOCKER_TEST_WORKCELL_GEOMETRY is not set";
  const DepthCameraIntrinsics intrinsics = wrist_intrinsics();

  // Recorded rather than glossed, because the tempting claim is that a missed product gives
  // itself away through its own shadow, and it does not. A ray that passes through a transparent
  // product and lands on the bed behind it is the same ray as one that crossed empty space, so
  // the shadow is lit, every bin has a return, and coverage is perfect on a lane that is not
  // empty. Nothing built on depth alone can separate the two; a see-through product is a limit of
  // this producer, not of its confidence.
  Product missed;
  missed.center_depth_m = 0.300;
  missed.radius_m = 0.033;
  missed.height_m = 0.122;
  missed.invisible = true;
  LaneScene lone(fixture->window, fixture->lane_half_span_m, fixture->divider_height_m);
  lone.add(missed);
  const auto measured = measure_lane_depth(
    render(lone, intrinsics, fixture->lane_from_optical), intrinsics.width, intrinsics,
    fixture->lane_from_optical, fixture->window, LaneDepthConfig{});
  ASSERT_TRUE(measured.has_value()) << measured.error().detail;
  EXPECT_DOUBLE_EQ(measured.value().available_depth_m, fixture->window.usable_depth_m);
  EXPECT_GT(measured.value().coverage, 0.99);

  // What does bound it: inside a packed column, everything in front of the missed product is
  // still measured, so the over-report is one product pitch rather than the whole lane.
  std::vector<Product> column = settled_column(fixture->window, 12U, 0.033, 0.122);
  column.back().invisible = true;
  LaneScene packed(fixture->window, fixture->lane_half_span_m, fixture->divider_height_m);
  for (const Product & product : column) {
    packed.add(product);
  }
  const auto packed_measured = measure_lane_depth(
    render(packed, intrinsics, fixture->lane_from_optical), intrinsics.width, intrinsics,
    fixture->lane_from_optical, fixture->window, LaneDepthConfig{});
  ASSERT_TRUE(packed_measured.has_value()) << packed_measured.error().detail;
  EXPECT_NEAR(
    packed_measured.value().available_depth_m -
    ground_truth_available_depth(fixture->window, column), 2.0 * 0.033, 0.002);
}

TEST(LaneDepthMeasurement, RaisesObstructedForAProductProtrudingFromTheLaneMouth) {
  const auto fixture = shipped_lane("lane_02");
  ASSERT_TRUE(fixture.has_value()) << "RESTOCKER_TEST_WORKCELL_GEOMETRY is not set";
  const DepthCameraIntrinsics intrinsics = wrist_intrinsics();

  // The shipped obstruction scenario: a small bottle jammed at lane depth 0.03, whose 0.034 m
  // radius puts its rear face 0.004 m behind the lane's rear entrance plane.
  Product jammed;
  jammed.center_depth_m = 0.030;
  jammed.radius_m = 0.034;
  jammed.height_m = 0.200;
  LaneScene scene(fixture->window, fixture->lane_half_span_m, fixture->divider_height_m);
  scene.add(jammed);
  const std::vector<float> depth = render(scene, intrinsics, fixture->lane_from_optical);
  const auto measured = measure_lane_depth(
    depth, intrinsics.width, intrinsics, fixture->lane_from_optical, fixture->window,
    LaneDepthConfig{});
  ASSERT_TRUE(measured.has_value()) << measured.error().detail;
  EXPECT_TRUE(measured.value().obstructed);
  EXPECT_GT(measured.value().obstruction_returns, 40U);

  LaneScene clear(fixture->window, fixture->lane_half_span_m, fixture->divider_height_m);
  clear.add(settled_column(fixture->window, 1U, 0.034, 0.200).front());
  const auto unobstructed = measure_lane_depth(
    render(clear, intrinsics, fixture->lane_from_optical), intrinsics.width, intrinsics,
    fixture->lane_from_optical, fixture->window, LaneDepthConfig{});
  ASSERT_TRUE(unobstructed.has_value()) << unobstructed.error().detail;
  EXPECT_FALSE(unobstructed.value().obstructed);
}

TEST(LaneDepthMeasurement, RefusesAWindowOrConfigurationItCannotUse) {
  LaneDepthWindow window;
  EXPECT_FALSE(restocker_perception::valid_lane_depth_window(window));
  window.lane_id = "lane_01";
  window.half_width_m = 0.1775;
  window.rear_clearance_m = 0.01;
  window.usable_depth_m = 0.85;
  window.usable_height_m = 0.40;
  window.bed_datum_depth_m = 0.84;
  window.front_inset_m = 0.010;
  window.bed_slope = 0.0699;
  window.floor_margin_m = 0.010;
  window.obstruction_reach_m = 0.060;
  EXPECT_TRUE(restocker_perception::valid_lane_depth_window(window));
  // A retainer behind the lane's own entrance leaves no window at all.
  LaneDepthWindow inverted = window;
  inverted.bed_datum_depth_m = 0.005;
  EXPECT_FALSE(restocker_perception::valid_lane_depth_window(inverted));

  LaneDepthConfig config;
  EXPECT_TRUE(restocker_perception::valid_lane_depth_config(config));
  config.pixel_stride = 0U;
  EXPECT_FALSE(restocker_perception::valid_lane_depth_config(config));
}

}  // namespace
