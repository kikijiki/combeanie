// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "restocker_gazebo/ground_truth_adapter_ros.hpp"

namespace restocker_gazebo
{
namespace
{

// One well-formed sample per whole second. The default cadence limit is 10 Hz, so consecutive
// whole seconds are always accepted and the return value reflects the publish decision only.
gz::msgs::Pose_V sample_at(std::int64_t seconds)
{
  gz::msgs::Pose_V sample;
  sample.mutable_header()->mutable_stamp()->set_sec(seconds);
  sample.mutable_header()->mutable_stamp()->set_nsec(0);
  gz::msgs::Pose * pose = sample.add_pose();
  pose->set_name("stock_can_01");
  pose->mutable_position()->set_x(0.0);
  pose->mutable_position()->set_y(0.0);
  pose->mutable_position()->set_z(0.7);
  pose->mutable_orientation()->set_w(1.0);
  return sample;
}

// Nothing publishes the pose topic here: every sample is handed to `deliver_pose_sample`
// directly, since these tests pin a lifetime rule and need no simulator.
class GroundTruthAdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {
          rclcpp::Parameter("scenario_config", std::string(RESTOCKER_TEST_SCENARIO)),
          rclcpp::Parameter("workcell_geometry", std::string(RESTOCKER_TEST_WORKCELL_GEOMETRY)),
          rclcpp::Parameter("product_catalog", std::string(RESTOCKER_TEST_PRODUCT_CATALOG)),
        });
    adapter_ = std::make_shared<GroundTruthAdapter>(options);
  }

  void TearDown() override
  {
    adapter_.reset();
  }

  std::shared_ptr<GroundTruthAdapter> adapter_;
};

TEST_F(GroundTruthAdapterTest, DeliversWhileOpen)
{
  // Without this the rest of the file could pass on an adapter that never publishes anything.
  EXPECT_TRUE(adapter_->deliver_pose_sample(sample_at(1)));
  EXPECT_TRUE(adapter_->deliver_pose_sample(sample_at(2)));
}

TEST_F(GroundTruthAdapterTest, DropsEverySampleOncePublishingIsClosed)
{
  ASSERT_TRUE(adapter_->deliver_pose_sample(sample_at(1)));

  adapter_->close_publishing();

  // Deterministic version of the recorded crash: the transport thread was publishing through the
  // lane publisher while the main thread had already finalised it in `~GroundTruthAdapter`.
  // After `close_publishing` no sample may reach a publisher.
  EXPECT_FALSE(adapter_->deliver_pose_sample(sample_at(2)));
  EXPECT_FALSE(adapter_->deliver_pose_sample(sample_at(3)));
  EXPECT_FALSE(adapter_->deliver_pose_sample(gz::msgs::Pose_V{}));
}

TEST_F(GroundTruthAdapterTest, ClosingPublishingIsIdempotent)
{
  adapter_->close_publishing();
  adapter_->close_publishing();
  EXPECT_FALSE(adapter_->deliver_pose_sample(sample_at(1)));
}

TEST_F(GroundTruthAdapterTest, NoDeliveryPublishesAfterCloseReturns)
{
  // A flag checked only on entry would let a delivery already past the check still be publishing
  // when the publishers are finalised. `deliver_pose_sample` therefore holds the publish lock
  // across its whole body, so a close cannot return while one is running. This drives that
  // concurrently; a delivery that began after the close returned must publish nothing.
  std::atomic<bool> closed{false};
  std::atomic<bool> stop{false};
  std::atomic<int> deliveries{0};
  std::atomic<int> published_after_close{0};

  std::thread producer(
    [&] {
      for (std::int64_t seconds = 1; !stop.load(); ++seconds) {
        const bool was_closed = closed.load();
        const bool published = adapter_->deliver_pose_sample(sample_at(seconds));
        if (was_closed && published) {
          ++published_after_close;
        }
        ++deliveries;
      }
    });

  while (deliveries.load() < 50) {
    std::this_thread::yield();
  }
  adapter_->close_publishing();
  closed.store(true);
  const int deliveries_at_close = deliveries.load();
  while (deliveries.load() < deliveries_at_close + 200) {
    std::this_thread::yield();
  }
  stop.store(true);
  producer.join();

  EXPECT_EQ(published_after_close.load(), 0);
}

}  // namespace
}  // namespace restocker_gazebo

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
