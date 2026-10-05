// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <vector>

#include "restocker_task_executor/tray_survey_logic.hpp"

namespace restocker_task_executor
{
namespace
{

using Observation = TrayObservation;

[[nodiscard]] Observation observation(
  double x, double y, double z, std::uint8_t product_class,
  float confidence = 0.8F, const std::string & sku = "", const std::string & backend =
  "wrist_rgbd_tray_overview", std::int32_t seconds = 10, std::uint32_t nanoseconds = 0U)
{
  Observation message;
  message.header.frame_id = "world";
  message.header.stamp.sec = seconds;
  message.header.stamp.nanosec = nanoseconds;
  message.product_class = product_class;
  message.confidence = confidence;
  if (!sku.empty()) {
    message.has_sku = true;
    message.sku = sku;
  }
  message.backend_name = backend;
  message.status = Observation::STATUS_OK;
  message.pose.pose.position.x = x;
  message.pose.pose.position.y = y;
  message.pose.pose.position.z = z;
  return message;
}

// -- window filtering ---------------------------------------------------------------------------

TEST(TraySurveyLogic, KeepsOnlyOkObservationsFromTheNamedBackendInsideTheHalfOpenWindow)
{
  std::vector<Observation> raw{
    observation(
      0.0, 0.0, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD",
      "wrist_rgbd_tray_overview", 10, 500'000'000U),
    // Stamped exactly at the start: half-open, so excluded (it predates the arm stopping).
    observation(
      0.1, 0.0, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD",
      "wrist_rgbd_tray_overview", 10, 0U),
    // Wrong backend: simulator ground truth sharing the topic cannot pass as a tray duty.
    observation(
      0.2, 0.0, 0.7, Observation::PRODUCT_CLASS_CAN, 1.0F, "SIM-CAN-STD",
      "gazebo_ground_truth", 11, 0U),
    // Not OK.
    observation(
      0.3, 0.0, 0.7, Observation::PRODUCT_CLASS_CAN, 0.1F, "SIM-CAN-STD",
      "wrist_rgbd_tray_overview", 11, 0U),
    observation(
      0.4, 0.0, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD",
      "wrist_rgbd_tray_overview", 12, 0U),
    // Beyond the window end.
    observation(
      0.5, 0.0, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD",
      "wrist_rgbd_tray_overview", 13, 0U)};
  raw[3].status = Observation::STATUS_INVALID_POSE;

  const std::vector<Observation> kept = observations_in_window(
    raw, "wrist_rgbd_tray_overview", 10'000'000'000LL, 12'000'000'000LL);
  ASSERT_EQ(kept.size(), 2U);
  EXPECT_DOUBLE_EQ(kept[0].pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(kept[1].pose.pose.position.x, 0.4);
}

TEST(TraySurveyLogic, TreatsAnEmptyStampAsUnusableRatherThanAsTheEpoch)
{
  Observation zero_stamp = observation(0.0, 0.0, 0.7, Observation::PRODUCT_CLASS_CAN);
  zero_stamp.header.stamp.sec = 0;
  zero_stamp.header.stamp.nanosec = 0U;
  EXPECT_FALSE(observation_stamp_ns(zero_stamp).has_value());

  const Observation stamped = observation(0.0, 0.0, 0.7, Observation::PRODUCT_CLASS_CAN);
  const auto nanoseconds = observation_stamp_ns(stamped);
  ASSERT_TRUE(nanoseconds.has_value());
  EXPECT_EQ(*nanoseconds, 10'000'000'000LL);
}

// -- merging ------------------------------------------------------------------------------------

TEST(TraySurveyLogic, MergesRepeatAcquisitionsOfOneProductAndKeepsDistinctProductsApart)
{
  const std::vector<Observation> station_one{
    observation(-0.42, -0.25, 0.63, Observation::PRODUCT_CLASS_CAN),
    observation(0.0, -0.25, 0.67, Observation::PRODUCT_CLASS_SMALL_BOTTLE)};
  // The same can re-estimated two millimetres away by a later frame, plus a second can that is a
  // genuinely distinct product (0.09 m away, beyond the merge radius).
  const std::vector<Observation> station_two{
    observation(-0.418, -0.25, 0.63, Observation::PRODUCT_CLASS_CAN, 0.9F),
    observation(-0.33, -0.25, 0.63, Observation::PRODUCT_CLASS_CAN, 0.7F)};

  const std::vector<Observation> merged =
    merge_candidates(station_one, station_two, 0.04);
  ASSERT_EQ(merged.size(), 3U);
  // First-seen order preserved: can, bottle; the second station replaced the can in place and
  // appended the distinct one.
  EXPECT_EQ(merged[0].product_class, Observation::PRODUCT_CLASS_CAN);
  EXPECT_FLOAT_EQ(merged[0].confidence, 0.9F);
  EXPECT_EQ(merged[1].product_class, Observation::PRODUCT_CLASS_SMALL_BOTTLE);
  EXPECT_NEAR(merged[2].pose.pose.position.x, -0.33, 1.0e-12);

  // A different class at the same place is never merged into it.
  const std::vector<Observation> cross_class = merge_candidates(
    station_one, {observation(-0.42, -0.25, 0.63, Observation::PRODUCT_CLASS_LARGE_BOTTLE)},
    0.04);
  EXPECT_EQ(cross_class.size(), 3U);
}

// -- selection ----------------------------------------------------------------------------------

TEST(TraySurveyLogic, SelectsByConfidenceThenPositionWithoutDependingOnArrivalOrder)
{
  const std::vector<Observation> candidates{
    observation(0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.75F),
    observation(-0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.75F),
    observation(0.0, -0.25, 0.7, Observation::PRODUCT_CLASS_LARGE_BOTTLE, 0.90F)};

  const auto any = select_candidate(candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "");
  ASSERT_TRUE(any.has_value());
  EXPECT_EQ(any->product_class, Observation::PRODUCT_CLASS_LARGE_BOTTLE);

  // Among equal confidence, the smallest X wins, whichever order the input arrives in.
  const auto cans_forward = select_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "");
  ASSERT_TRUE(cans_forward.has_value());
  EXPECT_NEAR(cans_forward->pose.pose.position.x, -0.30, 1.0e-12);

  std::vector<Observation> reversed(candidates.rbegin(), candidates.rend());
  const auto cans_reversed = select_candidate(
    reversed, Observation::PRODUCT_CLASS_CAN, false, "");
  ASSERT_TRUE(cans_reversed.has_value());
  EXPECT_NEAR(cans_reversed->pose.pose.position.x, -0.30, 1.0e-12);
}

TEST(TraySurveyLogic, AppliesTheClassAndSkuIntentAsSeparateFilters)
{
  const std::vector<Observation> candidates{
    observation(-0.3, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.9F, "SIM-CAN-STD"),
    observation(0.3, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.95F, "SIM-CAN-CITRUS"),
    observation(
      0.0, -0.25, 0.7, Observation::PRODUCT_CLASS_SMALL_BOTTLE, 0.99F,
      "SIM-BOTTLE-SMALL")};

  const auto cans = select_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "");
  ASSERT_TRUE(cans.has_value());
  EXPECT_EQ(cans->sku, "SIM-CAN-CITRUS");

  const auto citrus = select_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, true, "SIM-CAN-CITRUS");
  ASSERT_TRUE(citrus.has_value());
  EXPECT_EQ(citrus->sku, "SIM-CAN-CITRUS");

  // A SKU the candidates do not report matches nothing, it does not fall back to class only.
  const auto absent_sku = select_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, true, "SIM-DOES-NOT-EXIST");
  EXPECT_FALSE(absent_sku.has_value());

  const auto bottles = select_candidate(
    candidates, Observation::PRODUCT_CLASS_LARGE_BOTTLE, false, "");
  EXPECT_FALSE(bottles.has_value());
}

TEST(TraySurveyLogic, SkipsCandidatesMarkedRefutedWithinTheRadiusAndReselectsTheRest)
{
  const std::vector<Observation> candidates{
    observation(-0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.85F)};
  std::vector<geometry_msgs::msg::Point> refuted(1U);
  refuted[0].x = -0.30;
  refuted[0].y = -0.25;
  refuted[0].z = 0.7;

  // The marked candidate is the highest-confidence one; the mark must beat confidence.
  const auto reselection = select_candidate(
    candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", refuted, 0.04);
  ASSERT_TRUE(reselection.has_value());
  EXPECT_NEAR(reselection->pose.pose.position.x, 0.30, 1.0e-12);

  // A mark whose own point is outside the radius reaches neither candidate, so the original
  // winner stands: the exclusion is bounded by the radius, not by the mark list alone.
  std::vector<geometry_msgs::msg::Point> distant(1U);
  distant[0].x = 0.0;
  distant[0].y = 0.5;
  distant[0].z = 1.2;
  const auto far_mark = select_candidate(
    candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", distant, 0.01);
  ASSERT_TRUE(far_mark.has_value());
  EXPECT_NEAR(far_mark->pose.pose.position.x, -0.30, 1.0e-12);

  // A non-positive radius disables the mark entirely, for callers that mean "no exclusion".
  const auto disabled = select_candidate(
    candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", refuted, 0.0);
  ASSERT_TRUE(disabled.has_value());
  EXPECT_NEAR(disabled->pose.pose.position.x, -0.30, 1.0e-12);
}

TEST(TraySurveyLogic, EveryCandidateRefutedYieldsNoCandidateRatherThanTheDoomedOne)
{
  const std::vector<Observation> candidates{
    observation(-0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.85F)};
  std::vector<geometry_msgs::msg::Point> refuted(2U);
  refuted[0].x = -0.30;
  refuted[0].y = -0.25;
  refuted[0].z = 0.7;
  refuted[1].x = 0.30;
  refuted[1].y = -0.25;
  refuted[1].z = 0.7;

  const auto exhausted = select_candidate(
    candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", refuted, 0.04);
  EXPECT_FALSE(exhausted.has_value());
}

// -- feed order (Milestone 10 §6, Card 066) -----------------------------------------------------

// The dense scenario's workcell: shelf = world − (0, 0.55, 0.75), so a larger world y is a larger
// shelf y, the front of the tray (nearer the robot).
[[nodiscard]] FeedOrder dense_feed_order()
{
  FeedOrder feed;
  feed.shelf_from_planning = Eigen::Isometry3d(Eigen::Translation3d(0.0, -0.55, -0.75));
  return feed;
}

[[nodiscard]] geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point marked;
  marked.x = x;
  marked.y = y;
  marked.z = z;
  return marked;
}

// Card 063 dev run 2, cycle 6: the x = −0.15 can column after stock_can_01 left it. The survey
// walked it from the rear (smallest y first) and confirmed stock_can_05 while stock_can_03 stood
// in front, and the coordinator refused the pin. Only the front may be nominated, whatever
// confidence or the position tie-break say.
TEST(TraySurveyLogic, NominatesOnlyTheFrontOfATrayFeedColumn)
{
  const std::vector<Observation> column{
    observation(-0.15, -1.020, 0.631, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(-0.15, -0.935, 0.631, Observation::PRODUCT_CLASS_CAN, 0.90F),
    observation(-0.15, -0.850, 0.631, Observation::PRODUCT_CLASS_CAN, 0.90F),
    observation(-0.15, -0.765, 0.631, Observation::PRODUCT_CLASS_CAN, 0.80F)};

  // Control: the pre-066 order picks the rear can.
  const auto unordered = select_candidate(
    column, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04);
  ASSERT_TRUE(unordered.has_value());
  EXPECT_NEAR(unordered->pose.pose.position.y, -1.020, 1.0e-12);

  const CandidateSelection front = select_feed_front_candidate(
    column, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order());
  ASSERT_TRUE(front.selected.has_value());
  EXPECT_NEAR(front.selected->pose.pose.position.y, -0.765, 1.0e-12);
  EXPECT_EQ(front.feed_blocked_candidates, 3U);

  EXPECT_FALSE(stands_behind_in_feed_column(column[3], column, dense_feed_order()));
  EXPECT_TRUE(stands_behind_in_feed_column(column[2], column, dense_feed_order()));
  EXPECT_TRUE(stands_behind_in_feed_column(column[0], column, dense_feed_order()));
}

// A refutation excludes the front from nomination but never removes it from its column: the
// product behind it is not jumped over. A second column's front is still nominable; with no
// second column the survey has no candidate and says how many stood behind something.
TEST(TraySurveyLogic, ARefutedFrontKeepsItsColumnBlocked)
{
  std::vector<Observation> tray{
    observation(-0.15, -0.935, 0.631, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(-0.15, -0.850, 0.631, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(-0.15, -0.765, 0.631, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(0.00, -0.850, 0.631, Observation::PRODUCT_CLASS_CAN, 0.50F),
    observation(0.00, -0.765, 0.631, Observation::PRODUCT_CLASS_CAN, 0.50F)};
  const std::vector<geometry_msgs::msg::Point> refuted{point(-0.15, -0.765, 0.631)};

  const CandidateSelection other_column = select_feed_front_candidate(
    tray, Observation::PRODUCT_CLASS_CAN, false, "", refuted, 0.04, dense_feed_order());
  ASSERT_TRUE(other_column.selected.has_value());
  EXPECT_NEAR(other_column.selected->pose.pose.position.x, 0.00, 1.0e-12);
  EXPECT_NEAR(other_column.selected->pose.pose.position.y, -0.765, 1.0e-12);
  EXPECT_EQ(other_column.feed_blocked_candidates, 3U);

  tray.resize(3U);
  const CandidateSelection none = select_feed_front_candidate(
    tray, Observation::PRODUCT_CLASS_CAN, false, "", refuted, 0.04, dense_feed_order());
  EXPECT_FALSE(none.selected.has_value());
  // The refuted front itself is excluded by its mark, not by feed order, so it is not counted.
  EXPECT_EQ(none.feed_blocked_candidates, 2U);
}

// Anything standing in front blocks, whatever its class: the coordinator's rule counts every free
// tray product, and so does the straight grasp approach.
TEST(TraySurveyLogic, AProductOfAnotherClassInFrontBlocksTheColumn)
{
  const std::vector<Observation> tray{
    observation(0.43, -0.850, 0.631, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(0.43, -0.765, 0.660, Observation::PRODUCT_CLASS_SMALL_BOTTLE, 0.60F)};
  const CandidateSelection selection = select_feed_front_candidate(
    tray, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order());
  EXPECT_FALSE(selection.selected.has_value());
  EXPECT_EQ(selection.feed_blocked_candidates, 1U);
}

// Neighbouring columns do not block each other (the dense pitch is 0.15 m in x), a product
// behind does not block the one in front of it, and a second hypothesis of the same product
// (inside the merge radius, here another class at 2 cm) is not a product in front of it.
TEST(TraySurveyLogic, OnlyAProductAheadInTheSameColumnBlocks)
{
  const std::vector<Observation> tray{
    observation(-0.15, -0.850, 0.631, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(0.00, -0.680, 0.631, Observation::PRODUCT_CLASS_CAN, 0.50F),
    observation(-0.15, -0.935, 0.631, Observation::PRODUCT_CLASS_CAN, 0.50F),
    observation(-0.15, -0.830, 0.660, Observation::PRODUCT_CLASS_SMALL_BOTTLE, 0.30F)};
  const CandidateSelection selection = select_feed_front_candidate(
    tray, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order());
  ASSERT_TRUE(selection.selected.has_value());
  EXPECT_NEAR(selection.selected->pose.pose.position.x, -0.15, 1.0e-12);
  EXPECT_NEAR(selection.selected->pose.pose.position.y, -0.850, 1.0e-12);
  EXPECT_EQ(selection.feed_blocked_candidates, 1U);

  // Column membership is |Δx| ≤ half width: 22 mm apart shares a column, 23 mm does not.
  const std::vector<Observation> edge{
    observation(-0.150, -0.850, 0.631, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(-0.128, -0.765, 0.631, Observation::PRODUCT_CLASS_CAN, 0.10F)};
  EXPECT_TRUE(stands_behind_in_feed_column(edge[0], edge, dense_feed_order()));
  std::vector<Observation> outside = edge;
  outside[1].pose.pose.position.x = -0.127;
  EXPECT_FALSE(stands_behind_in_feed_column(outside[0], outside, dense_feed_order()));
  // "Ahead" needs more than the coordinator's 1 mm.
  std::vector<Observation> level = edge;
  level[1].pose.pose.position.x = -0.150;
  level[1].pose.pose.position.y = -0.8495;
  level[1].product_class = Observation::PRODUCT_CLASS_CAN;
  FeedOrder no_merge = dense_feed_order();
  no_merge.same_product_radius_m = 0.0;
  EXPECT_FALSE(stands_behind_in_feed_column(level[0], level, no_merge));
  // A candidate that is not STATUS_OK is not evidence of anything standing there.
  std::vector<Observation> invalid = edge;
  invalid[1].status = Observation::STATUS_INVALID_POSE;
  EXPECT_FALSE(stands_behind_in_feed_column(invalid[0], invalid, dense_feed_order()));
}

// Ahead is judged in the shelf frame, not the planning frame: with the shelf turned half a
// revolution the front is the most negative planning-frame y.
TEST(TraySurveyLogic, JudgesFeedOrderInTheShelfFrame)
{
  FeedOrder turned;
  turned.shelf_from_planning =
    Eigen::Isometry3d(Eigen::AngleAxisd(3.14159265358979323846, Eigen::Vector3d::UnitZ()));
  const std::vector<Observation> column{
    observation(0.20, 0.300, 0.631, Observation::PRODUCT_CLASS_CAN, 0.90F),
    observation(0.20, 0.215, 0.631, Observation::PRODUCT_CLASS_CAN, 0.90F)};
  const CandidateSelection selection = select_feed_front_candidate(
    column, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, turned);
  ASSERT_TRUE(selection.selected.has_value());
  EXPECT_NEAR(selection.selected->pose.pose.position.y, 0.215, 1.0e-12);
  EXPECT_EQ(selection.feed_blocked_candidates, 1U);
}

// With every candidate a front the rule changes nothing: the existing confidence/position order
// and the class, SKU and position-mark filters decide exactly as select_candidate does.
TEST(TraySurveyLogic, FeedFrontSelectionKeepsTheExistingOrderAmongFronts)
{
  const std::vector<Observation> fronts{
    observation(0.30, -0.765, 0.631, Observation::PRODUCT_CLASS_CAN, 0.80F, "SIM-CAN-STD"),
    observation(-0.15, -0.765, 0.631, Observation::PRODUCT_CLASS_CAN, 0.80F, "SIM-CAN-STD"),
    observation(0.00, -0.765, 0.631, Observation::PRODUCT_CLASS_CAN, 0.85F, "SIM-CAN-OTHER"),
    observation(0.60, -0.765, 0.690, Observation::PRODUCT_CLASS_LARGE_BOTTLE, 0.99F)};
  const std::vector<geometry_msgs::msg::Point> marks{point(0.00, -0.765, 0.631)};
  for (const bool has_sku : {false, true}) {
    for (const auto & excluded : {std::vector<geometry_msgs::msg::Point>{}, marks}) {
      const auto expected = select_candidate(
        fronts, Observation::PRODUCT_CLASS_CAN, has_sku, "SIM-CAN-STD", excluded, 0.04);
      const CandidateSelection actual = select_feed_front_candidate(
        fronts, Observation::PRODUCT_CLASS_CAN, has_sku, "SIM-CAN-STD", excluded, 0.04,
        dense_feed_order());
      ASSERT_EQ(expected.has_value(), actual.selected.has_value());
      EXPECT_NEAR(
        expected->pose.pose.position.x, actual.selected->pose.pose.position.x, 1.0e-12);
      EXPECT_EQ(actual.feed_blocked_candidates, 0U);
    }
  }
}

// -- feed order, Card 065's scenarios -----------------------------------------------------------
// Card 065 (cmbrear, work/cmbrear-065 red 5cafc29 / green 205e23b) designed the same front-only
// rule independently; the manager handed the rule to Card 066. These are its scenarios ported to
// this API. One is inverted on purpose: Card 065's own amendment (Backstage 508dc897) after its
// dev run 1 made a refuted front keep shadowing its queue.

// The dense can columns as the overview saw them in Card 063's dev run 2, cycle 6: four cans
// 0.085 m apart at x = −0.15 and three at x = 0.0, all at equal confidence, so the old
// tie-break (smallest Y) names the back of the tray.
[[nodiscard]] std::vector<Observation> dense_can_columns()
{
  std::vector<Observation> candidates;
  for (const double y : {-1.02, -0.935, -0.85, -0.765}) {
    candidates.push_back(observation(-0.15, y, 0.692, Observation::PRODUCT_CLASS_CAN, 0.9F));
  }
  for (const double y : {-0.935, -0.85, -0.765}) {
    candidates.push_back(observation(0.0, y, 0.692, Observation::PRODUCT_CLASS_CAN, 0.9F));
  }
  return candidates;
}

TEST(TraySurveyFeedQueue, NominatesTheFrontOfADenseColumnNotItsBack)
{
  const auto candidates = dense_can_columns();
  const auto unordered = select_candidate(candidates, Observation::PRODUCT_CLASS_CAN, false, "");
  ASSERT_TRUE(unordered.has_value());
  EXPECT_NEAR(unordered->pose.pose.position.y, -1.02, 1.0e-12);

  const CandidateSelection selection = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order());
  ASSERT_TRUE(selection.selected.has_value());
  EXPECT_NEAR(selection.selected->pose.pose.position.x, -0.15, 1.0e-12);
  EXPECT_NEAR(selection.selected->pose.pose.position.y, -0.765, 1.0e-12);
  // Three cans behind the front at x = −0.15 and two at x = 0.0.
  EXPECT_EQ(selection.feed_blocked_candidates, 5U);
}

TEST(TraySurveyFeedQueue, AFrontOfAnotherClassStillShadowsTheProductBehindIt)
{
  const std::vector<Observation> candidates{
    observation(0.0, -0.765, 0.77, Observation::PRODUCT_CLASS_SMALL_BOTTLE, 0.9F),
    observation(0.0, -0.85, 0.692, Observation::PRODUCT_CLASS_CAN, 0.9F)};
  const CandidateSelection cans = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order());
  EXPECT_FALSE(cans.selected.has_value());
  EXPECT_EQ(cans.feed_blocked_candidates, 1U);

  const CandidateSelection bottles = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_SMALL_BOTTLE, false, "", {}, 0.04,
    dense_feed_order());
  ASSERT_TRUE(bottles.selected.has_value());
  EXPECT_EQ(bottles.feed_blocked_candidates, 0U);
}

TEST(TraySurveyFeedQueue, ANeighbourInTheNextQueueDoesNotShadow)
{
  const std::vector<Observation> side_by_side{
    observation(0.15, -0.68, 0.692, Observation::PRODUCT_CLASS_CAN, 0.8F),
    observation(0.0, -0.765, 0.692, Observation::PRODUCT_CLASS_CAN, 0.9F)};
  const CandidateSelection separate = select_feed_front_candidate(
    side_by_side, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order());
  ASSERT_TRUE(separate.selected.has_value());
  EXPECT_NEAR(separate.selected->pose.pose.position.x, 0.0, 1.0e-12) << "confidence decides";
  EXPECT_EQ(separate.feed_blocked_candidates, 0U);

  const std::vector<Observation> same_queue{
    observation(0.014, -0.68, 0.692, Observation::PRODUCT_CLASS_CAN, 0.8F),
    observation(0.0, -0.765, 0.692, Observation::PRODUCT_CLASS_CAN, 0.9F)};
  const CandidateSelection shared = select_feed_front_candidate(
    same_queue, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order());
  ASSERT_TRUE(shared.selected.has_value());
  EXPECT_NEAR(shared.selected->pose.pose.position.y, -0.68, 1.0e-12);
  EXPECT_EQ(shared.feed_blocked_candidates, 1U);
}

// Inverted from Card 065's pre-amendment case. Its dev run 1: the close view refused the front
// small bottle of a column as absent three cycles running while it stood at its stocking
// position; each refusal unshadowed the bottle behind it, which was confirmed and then refused
// by the coordinator (object_unavailable ×6) until max_ncp_resurveys ran out.
TEST(TraySurveyFeedQueue, ARefutedFrontKeepsShadowingSoTheRearIsNeverConfirmed)
{
  const auto candidates = dense_can_columns();
  const std::vector<geometry_msgs::msg::Point> refuted{point(-0.15, -0.765, 0.692)};
  const CandidateSelection selection = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "", refuted, 0.04, dense_feed_order());
  ASSERT_TRUE(selection.selected.has_value());
  // The other column's front, never the can behind the refused one.
  EXPECT_NEAR(selection.selected->pose.pose.position.x, 0.0, 1.0e-12);
  EXPECT_NEAR(selection.selected->pose.pose.position.y, -0.765, 1.0e-12);
  EXPECT_EQ(selection.feed_blocked_candidates, 5U);

  // With the other column's front refused as well, nothing is nominable.
  const std::vector<geometry_msgs::msg::Point> both{
    point(-0.15, -0.765, 0.692), point(0.0, -0.765, 0.692)};
  const CandidateSelection none = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "", both, 0.04, dense_feed_order());
  EXPECT_FALSE(none.selected.has_value());
  EXPECT_EQ(none.feed_blocked_candidates, 5U);
}

TEST(TraySurveyFeedQueue, EveryMatchingCandidateShadowedYieldsNoCandidateNotAFallback)
{
  const std::vector<Observation> candidates{
    observation(-0.6, -0.68, 0.86, Observation::PRODUCT_CLASS_LARGE_BOTTLE, 0.7F),
    observation(-0.6, -0.765, 0.692, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(-0.6, -0.85, 0.692, Observation::PRODUCT_CLASS_CAN, 0.95F)};
  const CandidateSelection selection = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order());
  EXPECT_FALSE(selection.selected.has_value());
  EXPECT_EQ(selection.feed_blocked_candidates, 2U);
}

TEST(TraySurveyFeedQueue, AheadIsTheShelfFrontNotThePlanningFrameY)
{
  FeedOrder turned = dense_feed_order();
  turned.shelf_from_planning =
    Eigen::Isometry3d(Eigen::AngleAxisd(3.14159265358979323846, Eigen::Vector3d::UnitZ()));
  const std::vector<Observation> candidates{
    observation(0.0, -0.68, 0.692, Observation::PRODUCT_CLASS_CAN, 0.9F),
    observation(0.0, -0.765, 0.692, Observation::PRODUCT_CLASS_CAN, 0.8F)};
  const CandidateSelection selection = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, turned);
  ASSERT_TRUE(selection.selected.has_value());
  EXPECT_NEAR(selection.selected->pose.pose.position.y, -0.765, 1.0e-12);
  EXPECT_EQ(selection.feed_blocked_candidates, 1U);
}

// -- skip marks vs refutations (Card 058) ------------------------------------------------------

TEST(TraySurveyLogic, SkipMarksExcludeLikeRefutationsAndClassifyByKind)
{
  const std::vector<Observation> candidates{
    observation(-0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.85F)};
  std::vector<geometry_msgs::msg::Point> skip(1U);
  skip[0].x = -0.30;
  skip[0].y = -0.25;
  skip[0].z = 0.7;

  // A skip mark excludes its candidate exactly like a refutation: the far one wins.
  const auto reselection = select_candidate(
    candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", {}, 0.04, skip);
  ASSERT_TRUE(reselection.has_value());
  EXPECT_NEAR(reselection->pose.pose.position.x, 0.30, 1.0e-12);

  // Both candidates marked only by skip: selection fails and the cause names skip marks.
  std::vector<geometry_msgs::msg::Point> both_skip = skip;
  geometry_msgs::msg::Point far_skip;
  far_skip.x = 0.30;
  far_skip.y = -0.25;
  far_skip.z = 0.7;
  both_skip.push_back(far_skip);
  EXPECT_FALSE(
    select_candidate(
      candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", {}, 0.04, both_skip)
    .has_value());
  EXPECT_EQ(
    classify_no_candidate(
      candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", {}, both_skip, 0.04),
    NoCandidateCause::kSkipMarksOnly);
}

TEST(TraySurveyLogic, NoCandidateCauseAndDetailArePinnedPerExclusionKind)
{
  const std::vector<Observation> candidates{
    observation(-0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.95F),
    observation(0.30, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.85F)};
  std::vector<geometry_msgs::msg::Point> mark(1U);
  mark[0].x = -0.30;
  mark[0].y = -0.25;
  mark[0].z = 0.7;
  std::vector<geometry_msgs::msg::Point> other_mark(1U);
  other_mark[0].x = 0.30;
  other_mark[0].y = -0.25;
  other_mark[0].z = 0.7;

  // Nothing matched the request at all: no class-matching candidate (empty or wrong class).
  EXPECT_EQ(
    classify_no_candidate({}, Observation::PRODUCT_CLASS_CAN, false, "", {}, {}, 0.04),
    NoCandidateCause::kNoMatch);
  EXPECT_EQ(
    classify_no_candidate(
      candidates, Observation::PRODUCT_CLASS_LARGE_BOTTLE, false, "", mark, mark, 0.04),
    NoCandidateCause::kNoMatch);

  // Refutations only (the pre-058 wording must not change).
  EXPECT_EQ(
    classify_no_candidate(
      candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", {mark[0], other_mark[0]}, {},
      0.04),
    NoCandidateCause::kRefutationsOnly);

  // Skip marks only: both candidates skip-marked, no refutation.
  std::vector<geometry_msgs::msg::Point> both_skip{mark[0], other_mark[0]};
  EXPECT_EQ(
    classify_no_candidate(
      candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", {}, both_skip, 0.04),
    NoCandidateCause::kSkipMarksOnly);

  // Mixed: one candidate refuted, the other skip-marked.
  EXPECT_EQ(
    classify_no_candidate(
      candidates, Observation::PRODUCT_CLASS_UNKNOWN, false, "", mark, other_mark, 0.04),
    NoCandidateCause::kMixed);

  EXPECT_EQ(
    no_candidate_detail(NoCandidateCause::kNoMatch),
    "no overview candidate matched the requested selection");
  EXPECT_EQ(
    no_candidate_detail(NoCandidateCause::kSkipMarksOnly),
    "every overview candidate matching the request was excluded by an active skip mark");
  EXPECT_EQ(
    no_candidate_detail(NoCandidateCause::kRefutationsOnly),
    "every overview candidate matching the request was refuted earlier this cycle");
  EXPECT_EQ(
    no_candidate_detail(NoCandidateCause::kMixed),
    "every overview candidate matching the request was excluded by earlier refutations "
    "and active skip marks");
}


// -- feed order × skip marks (Card 066 × Card 058 integration) ---------------------------------

// I1: a skip mark keeps its product from being nominated exactly like a refutation, and a
// skip-marked front keeps blocking the product behind it. Taking only the refutations would
// re-nominate the skipped front at once; taking only select_candidate would nominate the rear.
TEST(TraySurveyFeedQueue, ASkipMarkedFrontStaysUnnominatedAndStillBlocksItsColumn)
{
  const auto candidates = dense_can_columns();
  const std::vector<geometry_msgs::msg::Point> skipped{point(-0.15, -0.765, 0.692)};
  const CandidateSelection selection = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order(),
    skipped);
  ASSERT_TRUE(selection.selected.has_value());
  EXPECT_NEAR(selection.selected->pose.pose.position.x, 0.0, 1.0e-12)
    << "the other column's front, never the skipped front nor the can behind it";
  EXPECT_NEAR(selection.selected->pose.pose.position.y, -0.765, 1.0e-12);
  // Three behind the skipped front at x = −0.15, two behind the front at x = 0.0; the skipped
  // front itself is excluded by its mark, not by feed order, so it is not counted.
  EXPECT_EQ(selection.feed_blocked_candidates, 5U);

  const std::vector<geometry_msgs::msg::Point> both_fronts{
    point(-0.15, -0.765, 0.692), point(0.0, -0.765, 0.692)};
  const CandidateSelection skip_only = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "", {}, 0.04, dense_feed_order(),
    both_fronts);
  EXPECT_FALSE(skip_only.selected.has_value());
  EXPECT_EQ(skip_only.feed_blocked_candidates, 5U);
  // Mixed: one front refuted, the other skip-marked.
  const CandidateSelection mixed = select_feed_front_candidate(
    candidates, Observation::PRODUCT_CLASS_CAN, false, "", {point(-0.15, -0.765, 0.692)}, 0.04,
    dense_feed_order(), {point(0.0, -0.765, 0.692)});
  EXPECT_FALSE(mixed.selected.has_value());
  EXPECT_EQ(mixed.feed_blocked_candidates, 5U);
}

// I3's geometry: the per-candidate front report the campaign's lift reads.
TEST(TraySurveyFeedQueue, ReportsWhichCandidatesAreFeedColumnFronts)
{
  auto candidates = dense_can_columns();
  candidates.back().status = Observation::STATUS_INVALID_POSE;
  const std::vector<bool> fronts = feed_column_fronts(candidates, dense_feed_order());
  ASSERT_EQ(fronts.size(), candidates.size());
  // x = −0.15: −1.02, −0.935, −0.85 behind; −0.765 the front.
  EXPECT_EQ(fronts, (std::vector<bool>{false, false, false, true, false, true, false}))
    << "an invalid observation is never a front, and it blocks nothing: x = 0.0's −0.85 leads";
}

// I6: a feed-blocked NO_CANDIDATE never reads "no overview candidate matched".
TEST(TraySurveyFeedQueue, ANoCandidateDetailNamesFeedOrderBeforeAnyMark)
{
  EXPECT_EQ(
    no_candidate_detail(NoCandidateCause::kNoMatch, 0U),
    "no overview candidate matched the requested selection");
  EXPECT_EQ(
    no_candidate_detail(NoCandidateCause::kNoMatch, 3U),
    "every remaining matching candidate stands behind another tray product in its feed column "
    "(3 feed-blocked)");
  EXPECT_EQ(
    no_candidate_detail(NoCandidateCause::kSkipMarksOnly, 2U),
    "every remaining matching candidate stands behind another tray product in its feed column "
    "(2 feed-blocked); the others are excluded by an active skip mark");
  EXPECT_EQ(
    no_candidate_detail(NoCandidateCause::kRefutationsOnly, 1U),
    "every remaining matching candidate stands behind another tray product in its feed column "
    "(1 feed-blocked); the others were refuted earlier this cycle");
  EXPECT_EQ(
    no_candidate_detail(NoCandidateCause::kMixed, 1U),
    "every remaining matching candidate stands behind another tray product in its feed column "
    "(1 feed-blocked); the others are excluded by earlier refutations and active skip marks");
}

// Review cmbrev066b N1: marked stock standing behind a front is counted on its own and named, so
// the campaign ends on the feed rung's named block rather than on "stock exhausted". Card 065's
// front1 shape in the x = 0.43 small-bottle column.
TEST(TraySurveyFeedQueue, MarkedStockBehindAFrontIsCountedAndNamed)
{
  const std::vector<Observation> column{
    observation(0.43, -0.680, 0.66, Observation::PRODUCT_CLASS_SMALL_BOTTLE, 0.9F),
    observation(0.43, -0.765, 0.66, Observation::PRODUCT_CLASS_SMALL_BOTTLE, 0.9F)};
  // Front refuted this cycle, rear skip-marked: nothing nominable, nothing unmarked behind.
  const CandidateSelection marked = select_feed_front_candidate(
    column, Observation::PRODUCT_CLASS_SMALL_BOTTLE, false, "", {point(0.43, -0.680, 0.66)}, 0.04,
    dense_feed_order(), {point(0.43, -0.765, 0.66)});
  EXPECT_FALSE(marked.selected.has_value());
  EXPECT_EQ(marked.feed_blocked_candidates, 0U);
  EXPECT_EQ(marked.marked_feed_blocked_candidates, 1U);
  EXPECT_EQ(
    marked.feed_block_example,
    "small bottle at (0.430, -0.765) stands behind small bottle at (0.430, -0.680)");

  // Unmarked rear behind a refuted front: the plain feed-blocked count, same example.
  const CandidateSelection unmarked = select_feed_front_candidate(
    column, Observation::PRODUCT_CLASS_SMALL_BOTTLE, false, "", {point(0.43, -0.680, 0.66)}, 0.04,
    dense_feed_order());
  EXPECT_EQ(unmarked.feed_blocked_candidates, 1U);
  EXPECT_EQ(unmarked.marked_feed_blocked_candidates, 0U);
  EXPECT_EQ(unmarked.feed_block_example, marked.feed_block_example);

  // A marked front with nothing ahead of it is not marked stock behind a front.
  const CandidateSelection marked_front = select_feed_front_candidate(
    {column[0]}, Observation::PRODUCT_CLASS_SMALL_BOTTLE, false, "", {}, 0.04, dense_feed_order(),
    {point(0.43, -0.680, 0.66)});
  EXPECT_EQ(marked_front.marked_feed_blocked_candidates, 0U);
  EXPECT_EQ(marked_front.feed_block_example, "");
}

// -- confirmation -------------------------------------------------------------------------------

TEST(TraySurveyLogic, ConfirmsWhenTheCloseViewAgreesWithTheHypothesis)
{
  const TraySurveyLogicConfig config;
  const Observation selected =
    observation(
    0.0, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD",
    "wrist_rgbd_tray_overview");
  const std::vector<Observation> confirm_window{
    observation(
      0.004, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.85F, "SIM-CAN-STD",
      "wrist_rgbd_tray_confirm")};

  const ConfirmationEvaluation evaluation = evaluate_confirmation(selected, confirm_window, config);
  EXPECT_EQ(evaluation.verdict, ConfirmationVerdict::kConfirmed);
  ASSERT_TRUE(evaluation.associated_observation.has_value());
  EXPECT_EQ(evaluation.associated_observation->backend_name, "wrist_rgbd_tray_confirm");
  EXPECT_LT(evaluation.associated_distance_m, config.association_radius_m);
  EXPECT_NE(evaluation.detail.find("agrees"), std::string::npos) << evaluation.detail;
}

TEST(TraySurveyLogic, RefutesAClassContradictionRatherThanConfirmingIt)
{
  const TraySurveyLogicConfig config;
  const Observation selected =
    observation(0.0, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD");
  const std::vector<Observation> confirm_window{
    observation(
      0.004, -0.25, 0.7, Observation::PRODUCT_CLASS_LARGE_BOTTLE, 0.85F,
      "SIM-BOTTLE-LARGE", "wrist_rgbd_tray_confirm")};

  const ConfirmationEvaluation evaluation = evaluate_confirmation(selected, confirm_window, config);
  EXPECT_EQ(evaluation.verdict, ConfirmationVerdict::kRefutedClassMismatch);
  ASSERT_TRUE(evaluation.associated_observation.has_value());
  EXPECT_EQ(
    evaluation.associated_observation->product_class, Observation::PRODUCT_CLASS_LARGE_BOTTLE);
}

TEST(TraySurveyLogic, LetsAContradictionWinOverAnAgreeingFrameAtTheSamePlace)
{
  // Fail-safe: one frame agrees and another, also inside the association radius, claims a
  // different class. The hypothesis is refused and the report names the contradiction; a
  // confirmation that out-voted a contradiction would be the wrong way round.
  const TraySurveyLogicConfig config;
  const Observation selected =
    observation(0.0, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD");
  const std::vector<Observation> confirm_window{
    observation(
      0.002, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.85F, "SIM-CAN-STD",
      "wrist_rgbd_tray_confirm"),
    observation(
      0.009, -0.25, 0.7, Observation::PRODUCT_CLASS_LARGE_BOTTLE, 0.85F,
      "SIM-BOTTLE-LARGE", "wrist_rgbd_tray_confirm")};

  const ConfirmationEvaluation evaluation = evaluate_confirmation(selected, confirm_window, config);
  EXPECT_EQ(evaluation.verdict, ConfirmationVerdict::kRefutedClassMismatch);
  ASSERT_TRUE(evaluation.associated_observation.has_value());
  EXPECT_EQ(
    evaluation.associated_observation->product_class, Observation::PRODUCT_CLASS_LARGE_BOTTLE);
}

TEST(TraySurveyLogic, RefutesAnSkuContradictionOfTheSameClass)
{
  const TraySurveyLogicConfig config;
  const Observation selected =
    observation(0.0, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD");
  const std::vector<Observation> confirm_window{
    observation(
      0.004, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.85F, "SIM-CAN-CITRUS",
      "wrist_rgbd_tray_confirm")};

  const ConfirmationEvaluation evaluation = evaluate_confirmation(selected, confirm_window, config);
  EXPECT_EQ(evaluation.verdict, ConfirmationVerdict::kRefutedSkuMismatch);
  EXPECT_NE(evaluation.detail.find("SIM-CAN-CITRUS"), std::string::npos) << evaluation.detail;
}

TEST(TraySurveyLogic, RefutesWhenTheCloseViewSawTheTrayButNotTheCandidate)
{
  const TraySurveyLogicConfig config;
  const Observation selected =
    observation(0.0, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD");
  // A neighbour well outside the association radius: the camera looked, and the candidate is
  // not where the overview said it was.
  const std::vector<Observation> confirm_window{
    observation(
      0.35, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.85F, "SIM-CAN-STD",
      "wrist_rgbd_tray_confirm")};

  const ConfirmationEvaluation evaluation = evaluate_confirmation(selected, confirm_window, config);
  EXPECT_EQ(evaluation.verdict, ConfirmationVerdict::kRefutedAbsent);
  EXPECT_FALSE(evaluation.associated_observation.has_value());
  ASSERT_TRUE(evaluation.nearest_observation.has_value());
  EXPECT_NE(evaluation.detail.find("nearest"), std::string::npos) << evaluation.detail;
}

TEST(TraySurveyLogic, AnEmptyDenseFrontIsConfirmedAsTheNeighbourOnePitchBehindIt)
{
  // Card 065 review note N1, the intended behaviour: the dense tray's column pitch (0.085 m) is
  // inside the 0.10 m association radius. When the nominated front is empty and the close view
  // publishes the neighbour one pitch back at its own place, CONFIRM confirms the candidate *as
  // that neighbour*. The confirmed observation is the neighbour's own (position, and downstream
  // its own identity): the campaign acts on the product that is really there, which is the
  // column's new front. Only a neighbour beyond the radius leaves the empty front ABSENT.
  const TraySurveyLogicConfig config;
  const Observation empty_front = observation(
    0.43, -0.765, 0.77, Observation::PRODUCT_CLASS_SMALL_BOTTLE, 0.8F, "SIM-BOTTLE-SMALL");
  const std::vector<Observation> neighbour_one_pitch_back{
    observation(
      0.43, -0.85, 0.77, Observation::PRODUCT_CLASS_SMALL_BOTTLE, 0.85F, "SIM-BOTTLE-SMALL",
      "wrist_rgbd_tray_confirm")};
  const ConfirmationEvaluation confirmed =
    evaluate_confirmation(empty_front, neighbour_one_pitch_back, config);
  EXPECT_EQ(confirmed.verdict, ConfirmationVerdict::kConfirmed);
  ASSERT_TRUE(confirmed.associated_observation.has_value());
  EXPECT_DOUBLE_EQ(confirmed.associated_observation->pose.pose.position.y, -0.85)
    << "the confirmation carries the neighbour's own measured position, never the candidate's";
  EXPECT_NEAR(confirmed.associated_distance_m, 0.085, 1.0e-9);

  const std::vector<Observation> neighbour_beyond_the_radius{
    observation(
      0.43, -0.87, 0.77, Observation::PRODUCT_CLASS_SMALL_BOTTLE, 0.85F, "SIM-BOTTLE-SMALL",
      "wrist_rgbd_tray_confirm")};
  EXPECT_EQ(
    evaluate_confirmation(empty_front, neighbour_beyond_the_radius, config).verdict,
    ConfirmationVerdict::kRefutedAbsent);
}

TEST(TraySurveyLogic, ReportsNoObservationAsItsOwnVerdictNotAsARefutation)
{
  const TraySurveyLogicConfig config;
  const Observation selected =
    observation(0.0, -0.25, 0.7, Observation::PRODUCT_CLASS_CAN, 0.8F, "SIM-CAN-STD");

  const ConfirmationEvaluation evaluation = evaluate_confirmation(selected, {}, config);
  EXPECT_EQ(evaluation.verdict, ConfirmationVerdict::kNoObservation);
  EXPECT_FALSE(evaluation.associated_observation.has_value());
  EXPECT_FALSE(evaluation.nearest_observation.has_value());
}

// Card 064: the zero-dwell drain names the stage from the drained report's counts
// (Milestone 10 Stage 4). Each row of TrayStationAcquisition.msg's stage table is one case.
[[nodiscard]] restocker_interfaces::msg::TrayStationAcquisition drained_report(
  std::uint32_t images, std::uint32_t detection_frames, std::uint32_t frames_with_detections,
  std::uint32_t published, std::uint32_t admitted, std::uint32_t late_images = 0U,
  std::uint32_t late_published = 0U)
{
  restocker_interfaces::msg::TrayStationAcquisition report;
  report.station = "tray_1";
  report.image_tap_configured = true;
  report.detection_tap_configured = true;
  report.images = images;
  report.detection_frames = detection_frames;
  report.frames_with_detections = frames_with_detections;
  report.published = published;
  report.admitted = admitted;
  report.drained = true;
  report.late_images = late_images;
  report.late_published = late_published;
  return report;
}

[[nodiscard]] bool starts_with(const std::string & text, const std::string & prefix)
{
  return text.rfind(prefix, 0) == 0;
}

TEST(TraySurveyDrainClassification, LateAdmittedFramesAreADelivery)
{
  const std::string text = classify_drained_station(drained_report(6, 6, 6, 6, 6, 0, 6));
  EXPECT_TRUE(starts_with(text, "late delivery:")) << text;
}

TEST(TraySurveyDrainClassification, NoImageInTheWindowIsAnInputGap)
{
  const std::string text = classify_drained_station(drained_report(0, 0, 0, 0, 0));
  EXPECT_TRUE(starts_with(text, "input gap:")) << text;
}

TEST(TraySurveyDrainClassification, ImagesWithoutDetectionFramesNameTheDuty)
{
  const std::string text = classify_drained_station(drained_report(6, 0, 0, 0, 0));
  EXPECT_TRUE(starts_with(text, "duty not processing:")) << text;
}

TEST(TraySurveyDrainClassification, NoProposalIsAnEmptyViewNotAStageGap)
{
  // SC-004 slot 20's terminal stations: the colour stage looked and found nothing.
  const std::string text = classify_drained_station(drained_report(6, 6, 0, 0, 0));
  EXPECT_TRUE(starts_with(text, "empty view: the colour stage found nothing")) << text;
  EXPECT_EQ(text.find("stage gap"), std::string::npos) << text;
}

TEST(TraySurveyDrainClassification, EveryProposalRefusedIsAnEmptyViewNotAStageGap)
{
  // SC-004 slot 14's terminal tray_1: six frames with proposals, the geometry gates refused all.
  const std::string text = classify_drained_station(drained_report(6, 6, 6, 0, 0));
  EXPECT_TRUE(starts_with(text, "empty view: estimation refused every proposal")) << text;
  EXPECT_EQ(text.find("stage gap"), std::string::npos) << text;
}

TEST(TraySurveyDrainClassification, LateImagesDoNotTurnARefusalIntoAnAdmissionMismatch)
{
  // SC-004 slot 07: an image arrived during the drain, nothing was ever published. The old label
  // said "none met the admission rules" although there was nothing to admit.
  const std::string text = classify_drained_station(drained_report(6, 5, 5, 0, 0, 1, 0));
  EXPECT_TRUE(starts_with(text, "empty view: estimation refused every proposal")) << text;
  EXPECT_EQ(text.find("admission"), std::string::npos) << text;
}

TEST(TraySurveyDrainClassification, PublishedButNotAdmittedIsAnAdmissionMismatch)
{
  const std::string text = classify_drained_station(drained_report(6, 6, 6, 6, 0));
  EXPECT_TRUE(starts_with(text, "admission mismatch:")) << text;
}

TEST(TraySurveyDrainClassification, AnUnconfiguredDetectionTapDoesNotClaimAStage)
{
  auto report = drained_report(6, 0, 0, 0, 0);
  report.detection_tap_configured = false;
  const std::string text = classify_drained_station(report);
  EXPECT_TRUE(starts_with(text, "nothing published:")) << text;
  EXPECT_NE(text.find("detection tap is not configured"), std::string::npos) << text;
}

TEST(TraySurveyDrainClassification, AnUnconfiguredImageTapIsNotAnInputGap)
{
  auto report = drained_report(0, 6, 6, 0, 0);
  report.image_tap_configured = false;
  const std::string text = classify_drained_station(report);
  EXPECT_TRUE(starts_with(text, "empty view: estimation refused every proposal")) << text;
}

// Milestone 10 §6, Card 069: absence probes. The camera sits at the planning-frame origin
// looking along +Z (optical convention), so a probe at (0, 0, 1) is dead centre at 1 m.
AbsenceProbeConfig absence_config()
{
  AbsenceProbeConfig config;
  config.frustum.width_px = 1280U;
  config.frustum.height_px = 720U;
  config.frustum.horizontal_fov_rad = 1.48;
  config.frustum.near_clip_m = 0.05;
  config.frustum.far_clip_m = 2.5;
  config.seen_radius_m = 0.10;
  config.occluder_radius_m = 0.05;
  config.framing_margin_px = 20.0;
  return config;
}

AbsenceStationView station_at_origin(const bool healthy)
{
  AbsenceStationView view;
  view.healthy = healthy;
  return view;
}

restocker_interfaces::msg::TrayStationAcquisition healthy_report()
{
  restocker_interfaces::msg::TrayStationAcquisition report;
  report.image_tap_configured = true;
  report.detection_tap_configured = true;
  report.images = 6U;
  report.detection_frames = 6U;
  report.frames_with_detections = 6U;
  report.published = 6U;
  report.admitted = 6U;
  return report;
}

TEST(AbsenceProbe, HealthRequiresAStationThatDemonstrablySeesProducts)
{
  EXPECT_TRUE(station_acquisition_healthy(healthy_report()));
  auto blind = healthy_report();
  blind.frames_with_detections = 0U;
  blind.published = 0U;
  blind.admitted = 0U;
  EXPECT_FALSE(station_acquisition_healthy(blind))
    << "a detector that found nothing at all looks exactly like an empty tray (review N2)";
  auto dropped = healthy_report();
  dropped.published = 0U;
  dropped.admitted = 0U;
  EXPECT_FALSE(station_acquisition_healthy(dropped))
    << "estimation dropped every proposal: a refused present product looks the same";
  auto refused_at_admission = healthy_report();
  refused_at_admission.admitted = 0U;
  EXPECT_FALSE(station_acquisition_healthy(refused_at_admission))
    << "published but none admitted (backend or status mismatch) is not evidence (review N1)";
  auto no_images = healthy_report();
  no_images.images = 0U;
  EXPECT_FALSE(station_acquisition_healthy(no_images));
  auto no_detector = healthy_report();
  no_detector.detection_frames = 0U;
  EXPECT_FALSE(station_acquisition_healthy(no_detector));
  auto no_tap = healthy_report();
  no_tap.detection_tap_configured = false;
  EXPECT_FALSE(station_acquisition_healthy(no_tap));
}

TEST(AbsenceProbe, ACandidateOfAnyClassNearTheProbeIsSeen)
{
  const Eigen::Vector3d probe(0.0, 0.0, 1.0);
  // Within the 0.10 m seen radius but beyond the 0.04 m merge radius: a disturbed product.
  EXPECT_EQ(
    classify_absence_probe(
      probe, {Eigen::Vector3d(0.08, 0.0, 1.0)}, {Eigen::Vector3d(0.08, 0.0, 1.0)},
      {station_at_origin(true)}, absence_config()),
    AbsenceProbeVerdict::kSeen);
  EXPECT_EQ(
    classify_absence_probe(
      probe, {Eigen::Vector3d(0.02, 0.0, 1.0)}, {Eigen::Vector3d(0.02, 0.0, 1.0)},
      {station_at_origin(true)}, absence_config()),
    AbsenceProbeVerdict::kSeen);
}

TEST(AbsenceProbe, AHealthyClearFramingWithNothingThereIsViewedEmpty)
{
  const Eigen::Vector3d probe(0.0, 0.0, 1.0);
  EXPECT_EQ(
    classify_absence_probe(
      probe, {Eigen::Vector3d(0.3, 0.0, 1.0)}, {Eigen::Vector3d(0.3, 0.0, 1.0)},
      {station_at_origin(true)}, absence_config()),
    AbsenceProbeVerdict::kViewedEmpty);
}

TEST(AbsenceProbe, ANearerPointOnTheLineOfSightOccludes)
{
  const Eigen::Vector3d probe(0.0, 0.0, 1.0);
  const std::vector<Eigen::Vector3d> occluders{Eigen::Vector3d(0.01, 0.0, 0.6)};
  EXPECT_EQ(
    classify_absence_probe(probe, {}, occluders, {station_at_origin(true)}, absence_config()),
    AbsenceProbeVerdict::kOccluded);
  // Beside the ray, or behind the probe, does not block.
  EXPECT_EQ(
    classify_absence_probe(
      probe, {}, {Eigen::Vector3d(0.2, 0.0, 0.6), Eigen::Vector3d(0.0, 0.0, 1.5)},
      {station_at_origin(true)}, absence_config()),
    AbsenceProbeVerdict::kViewedEmpty);
  // A second station with a clear view is enough.
  AbsenceStationView side = station_at_origin(true);
  side.world_from_optical.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
  EXPECT_EQ(
    classify_absence_probe(
      probe, {}, occluders, {station_at_origin(true), side}, absence_config()),
    AbsenceProbeVerdict::kViewedEmpty);
}

TEST(AbsenceProbe, OutsideEveryFrustumIsNotCovered)
{
  EXPECT_EQ(
    classify_absence_probe(
      Eigen::Vector3d(0.0, 0.0, -1.0), {}, {}, {station_at_origin(true)}, absence_config()),
    AbsenceProbeVerdict::kNotCovered);
  EXPECT_EQ(
    classify_absence_probe(
      Eigen::Vector3d(5.0, 0.0, 1.0), {}, {}, {station_at_origin(true)}, absence_config()),
    AbsenceProbeVerdict::kNotCovered);
  EXPECT_EQ(
    classify_absence_probe(Eigen::Vector3d(0.0, 0.0, 1.0), {}, {}, {}, absence_config()),
    AbsenceProbeVerdict::kNotCovered);
}

TEST(AbsenceProbe, FramedOnlyByAnUnhealthyStationIsNoEvidence)
{
  EXPECT_EQ(
    classify_absence_probe(
      Eigen::Vector3d(0.0, 0.0, 1.0), {}, {}, {station_at_origin(false)}, absence_config()),
    AbsenceProbeVerdict::kNoEvidence);
}

}  // namespace
}  // namespace restocker_task_executor
