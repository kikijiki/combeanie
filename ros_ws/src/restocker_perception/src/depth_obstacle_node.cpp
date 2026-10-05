// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <Eigen/Geometry>

#include <span>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/obstacle_observation.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "restocker_perception/depth_obstacle_extraction.hpp"

namespace restocker_perception
{

// Turns each depth image into the axis-aligned boxes that are not accounted for by the modelled
// workcell or by the robot itself.
//
// The node publishes one observation per depth image and never accumulates across images: an
// accumulated map is assembled from many instants, so a consumer could not say which instant it
// describes. One image, one stamp, one set of boxes; the consumer applies its own freshness rule
// to that stamp.
class DepthObstacleNode final : public rclcpp::Node
{
public:
  DepthObstacleNode()
  : Node("depth_obstacle_detector"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "world");
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/overhead_camera/depth_image");
    const auto camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/overhead_camera/camera_info");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/perception/obstacle_observations");
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.25);
    // Images older than this are dropped rather than resolved against a transform from a
    // different instant.
    max_image_age_sec_ = declare_parameter<double>("max_image_age_sec", 1.0);
    // How long the stream may be silent before the heartbeat reports it. Same default as the
    // projector's obstacle_max_age_sec, so the report starts when certification is at risk.
    heartbeat_silence_sec_ = declare_parameter<double>("heartbeat_silence_sec", 2.0);

    config_.min_depth_m = declare_parameter<double>("min_depth_m", 0.20);
    config_.max_depth_m = declare_parameter<double>("max_depth_m", 4.0);
    config_.pixel_stride =
      static_cast<std::uint32_t>(declare_parameter<std::int64_t>("pixel_stride", 4));
    config_.voxel_size_m = declare_parameter<double>("voxel_size_m", 0.02);
    config_.min_cluster_voxels =
      static_cast<std::size_t>(declare_parameter<std::int64_t>("min_cluster_voxels", 12));
    config_.min_box_extent_m = declare_parameter<double>("min_box_extent_m", 0.04);
    config_.max_box_extent_m = declare_parameter<double>("max_box_extent_m", 1.2);
    config_.max_boxes = static_cast<std::size_t>(declare_parameter<std::int64_t>("max_boxes", 8));
    config_.volume_min = to_vector(
      declare_parameter<std::vector<double>>(
        "volume_of_interest_min_xyz_m", std::vector<double>{-1.8, -1.6, 0.05}),
      "volume_of_interest_min_xyz_m");
    config_.volume_max = to_vector(
      declare_parameter<std::vector<double>>(
        "volume_of_interest_max_xyz_m", std::vector<double>{1.8, 0.20, 1.60}),
      "volume_of_interest_max_xyz_m");

    config_.cluster_vertical_gap_voxels =
      declare_parameter<std::int64_t>("cluster_vertical_gap_voxels", 4);

    // The robot, as one capsule per physical link. The rail values come from its local primitive
    // solids. The arm values enclose the transformed vertices of each official ur_description
    // 3.5.1 UR10e collision STL, rounded out to the millimetre. The long upper-arm and forearm
    // meshes remain capsules so link length does not become false lateral clearance. Camera
    // sampling error is separate, in self_filter_margin_m.
    //
    // Per link, in that link's own frame:
    //   rail_base       (-2.0,0,0.08)..(2.0,0,0.08)  0.145  4.0 x 0.24 x 0.16 box: the segment is
    //                     the box's long axis, the radius its cross-section half-diagonal
    //                     hypot(0.12, 0.08) = 0.1443.
    //   carriage        (-0.25,0,0.06)..(0.25,0,0.06) 0.238  0.50 x 0.46 x 0.12 box, along its
    //                     long axis: hypot(0.23, 0.06) = 0.2377. Along z it would need
    //                     hypot(0.25, 0.23) = 0.340.
    //   base_link_inertia  (0.0027,-0.0022,0)..(0.0027,-0.0022,0.0993)             0.099
    //   shoulder_link     (0.0009,0.0108,-0.0817)..(0.0009,0.0108,0.0939)          0.130
    //   upper_arm_link    (-0.6724,0,0.2220)..(0.0767,0,0.2220)                    0.148
    //   forearm_link      (-0.6177,-0.0003,0.0240)..(0.0588,-0.0003,0.0240)        0.095
    //   wrist_1_link      (-0.0008,-0.0666,0.0025)..(-0.0008,0.0576,0.0025)        0.085
    //   wrist_2_link      (0.0009,-0.0570,0.0028)..(0.0009,0.0680,0.0028)          0.073
    //   wrist_3_link      (0.0004,-0.0521,-0.0158)..(0.0004,0.0454,-0.0158)        0.056
    //   gripper         (0,0,0)..(0,0,0.285)  0.098  the radius is the body box, hypot(0.08,
    //                     0.055) = 0.0971, which dominates the open fingers' hypot(0.0175,
    //                     0.074) = 0.076. The segment reaches to 0.285 to cover the largest
    //                     catalogued product held at the grasp centre: bottle.large is 0.290 m
    //                     tall and rides 0.14 m out on grasp_center, spanning -0.005..0.285 at
    //                     0.045 m radius. A held product outside the filter would come back as an
    //                     obstacle inside the gripper and make the insert unplannable.
    //   wrist_camera_link (0,-0.045,0)..(0,0.045,0)                           0.033
    //                     the 0.05 x 0.09 x 0.04 housing on tool0 at (0.11, 0, 0.07): the
    //                     segment spans its long axis, the radius its half-section
    //                     hypot(0.025, 0.02) = 0.0320, rounded up. The housing corners reach
    //                     0.138 m from the gripper capsule axis against the 0.118 m that
    //                     capsule covers, so without this entry the overhead depth reports the
    //                     camera itself as an obstacle contacting gripper, wrist_camera_link and
    //                     left_finger — blocking tray-station IK and approach lines whenever
    //                     obstacle_perception composes this node (Card 023).
    //
    self_filter_frames_ = declare_parameter<std::vector<std::string>>(
      "self_filter_frames",
      std::vector<std::string>{
        "rail_base", "carriage", "base_link_inertia", "shoulder_link", "upper_arm_link",
        "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "gripper",
        "wrist_camera_link"});
    // Six values per frame: the capsule's segment start xyz then its end xyz, in that link's frame.
    self_filter_segments_ = declare_parameter<std::vector<double>>(
      "self_filter_segments_xyz_m",
      std::vector<double>{
        -2.00, 0.0, 0.08, 2.00, 0.0, 0.08,
        -0.25, 0.0, 0.06, 0.25, 0.0, 0.06,
        0.0027, -0.0022, 0.0, 0.0027, -0.0022, 0.0993,
        0.0009, 0.0108, -0.0817, 0.0009, 0.0108, 0.0939,
        -0.6724, 0.0, 0.2220, 0.0767, 0.0, 0.2220,
        -0.6177, -0.0003, 0.0240, 0.0588, -0.0003, 0.0240,
        -0.0008, -0.0666, 0.0025, -0.0008, 0.0576, 0.0025,
        0.0009, -0.0570, 0.0028, 0.0009, 0.0680, 0.0028,
        0.0004, -0.0521, -0.0158, 0.0004, 0.0454, -0.0158,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.285,
        0.0, -0.045, 0.0, 0.0, 0.045, 0.0});
    self_filter_radii_ = declare_parameter<std::vector<double>>(
      "self_filter_radii_m",
      std::vector<double>{
        0.145, 0.238, 0.099, 0.130, 0.148, 0.095, 0.085, 0.073, 0.056, 0.098, 0.033});
    // Added to every radius above. It is not slack in the robot model but the depth sensor's
    // inability to place a return on its source surface: at a silhouette the pixel straddles the
    // edge and its range interpolates between the link and whatever is behind it, so the
    // reconstructed point lands outside the solid it belongs to.
    //
    // 0.020 m is the camera sampling allowance for this overhead sensor, independent of arm
    // dimensions. It costs obstacle sensitivity only within 0.020 m of the arm's own surface,
    // inside the clearance any plan already keeps.
    self_filter_margin_m_ = declare_parameter<double>("self_filter_margin_m", 0.020);
    // How far back the same links are looked up to decide whether the robot was still. A window
    // shorter than one control period cannot see motion; one much longer would call the robot
    // moving long after it stopped.
    motion_probe_sec_ = declare_parameter<double>("motion_probe_sec", 0.20);
    motion_probe_tolerance_m_ = declare_parameter<double>("motion_probe_tolerance_m", 0.004);

    validate_configuration();

    // Reliable, unlike the depth stream it is derived from. This is a low-rate safety observation
    // whose absence puts the planning-scene projector into a degraded state, and a best-effort
    // publisher is invisible to any reliable subscriber (every default-QoS consumer and
    // diagnostic tool).
    observation_publisher_ = create_publisher<restocker_interfaces::msg::ObstacleObservation>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    // Reliability of the depth subscription. Best-effort is usual for a camera feed, but here the
    // newest depth image certifies the planning scene and the certification expires by wall age,
    // so a dropped image is a step towards an uncertified scene. These images are 2.1 to 3.7 MB
    // and best-effort loses them in proportion to their size.
    //
    // The history is shallow: `max_image_age_sec` drops any image whose stamp has aged past it, so
    // a deep reliable queue would only turn transport backlog into images this node discards.
    // The last few give the transport room to retransmit without hoarding.
    //
    // Measured over six paired rounds of the composed pipeline,
    // best-effort delivered 23% to 66% of the
    // emitted frames with gaps up to 2.822 s, past the projector's 2.0 s certification window, and
    // the projector refused to certify 24 times (observation ages up to 4.968 s). Reliable
    // delivered every emitted frame in all twelve reliable runs (2794 consecutive intervals, none
    // wider than one 0.167 s frame period, no degraded status), including runs against a GPU 85%
    // held by an unrelated process. The paired real-time-factor difference is +0.03 with a 95%
    // interval of [-0.07, +0.13], which rules out the 0.75-to-0.56 cost this was believed to
    // carry, though not a cost below about 0.10.
    //
    // The parameter exists because a bridge that publishes these best-effort is invisible to a
    // reliable subscription (silence, not degradation) and must be correctable without a rebuild.
    // See restocker_bringup's `obstacle_depth_reliable` launch argument.
    const bool reliable_depth = declare_parameter<bool>("reliable_sensor_qos", true);
    const auto sensor_qos = reliable_depth ?
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable() : rclcpp::QoS(rclcpp::SensorDataQoS());
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        camera_info_ = std::move(info);
      });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr image) {on_depth_image(*image);});

    RCLCPP_INFO(
      get_logger(), "publishing unmodeled obstacles on %s in frame %s", output_topic.c_str(),
      planning_frame_.c_str());

    // Silence accounting. The projector refuses the scene when the newest observation passes
    // obstacle_max_age_sec, so "the stream stopped" and "the node is not being scheduled" must be
    // distinguishable from an ordinary log: this heartbeat names the gap, says whether depth
    // images are still arriving behind it, and carries the drop counters that say why nothing was
    // published. It fires only while the stream is silent, so a healthy run logs nothing.
    last_image_steady_ = std::chrono::steady_clock::now();
    last_publish_steady_ = last_image_steady_;
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&DepthObstacleNode::on_heartbeat, this));
  }

private:
  using Observation = restocker_interfaces::msg::ObstacleObservation;

  [[nodiscard]] static Eigen::Vector3d to_vector(
    const std::vector<double> & values, const std::string & name)
  {
    if (values.size() != 3U) {
      throw std::invalid_argument(name + " must carry exactly three values");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  void validate_configuration() const
  {
    if (planning_frame_.empty()) {
      throw std::invalid_argument("planning_frame must be set");
    }
    if (self_filter_frames_.size() != self_filter_radii_.size()) {
      throw std::invalid_argument(
              "self_filter_frames and self_filter_radii_m must have the same length");
    }
    if (self_filter_segments_.size() != 6U * self_filter_frames_.size()) {
      throw std::invalid_argument(
              "self_filter_segments_xyz_m must carry six values per self-filter frame");
    }
    if (std::ranges::any_of(
        self_filter_segments_, [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument("self_filter_segments_xyz_m must all be finite");
    }
    if (std::ranges::any_of(
        self_filter_frames_, [](const std::string & frame) {return frame.empty();}))
    {
      throw std::invalid_argument("self_filter_frames must not contain an empty frame");
    }
    if (std::ranges::any_of(
        self_filter_radii_, [](double radius) {
          return !std::isfinite(radius) || radius <= 0.0;
        }))
    {
      throw std::invalid_argument("self_filter_radii_m must all be positive");
    }
    if (!std::isfinite(self_filter_margin_m_) || self_filter_margin_m_ < 0.0) {
      throw std::invalid_argument("self_filter_margin_m must be finite and not negative");
    }
    if (!std::isfinite(transform_timeout_sec_) || transform_timeout_sec_ < 0.0 ||
      !std::isfinite(max_image_age_sec_) || max_image_age_sec_ <= 0.0 ||
      !std::isfinite(heartbeat_silence_sec_) || heartbeat_silence_sec_ <= 0.0 ||
      !std::isfinite(motion_probe_sec_) || motion_probe_sec_ <= 0.0 ||
      !std::isfinite(motion_probe_tolerance_m_) || motion_probe_tolerance_m_ < 0.0)
    {
      throw std::invalid_argument("depth obstacle timing parameters are invalid");
    }
    if (!valid_depth_obstacle_config(config_)) {
      throw std::invalid_argument("depth obstacle extraction parameters are invalid");
    }
  }

  [[nodiscard]] std::chrono::nanoseconds transform_timeout() const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(transform_timeout_sec_));
  }

  // Times every frame that reaches the executor, including the ones the checks below drop, so
  // "no observation" can be told apart from "frames arriving but costing more than they arrive".
  void on_depth_image(const sensor_msgs::msg::Image & image)
  {
    const auto frame_started = std::chrono::steady_clock::now();
    last_image_steady_ = frame_started;
    ++images_received_;
    process_depth_image(image);
    const auto frame_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - frame_started);
    last_frame_ms_ = frame_ms;
    if (frame_ms > worst_frame_ms_) {
      worst_frame_ms_ = frame_ms;
    }
  }

  void process_depth_image(const sensor_msgs::msg::Image & image)
  {
    if (!camera_info_) {
      ++dropped_no_camera_info_;
      throttled_warning("no camera_info received yet; cannot unproject depth");
      return;
    }
    if (image.encoding != "32FC1") {
      ++dropped_invalid_image_;
      throttled_warning("depth image encoding " + image.encoding + " is not 32FC1");
      return;
    }
    if (camera_info_->width != image.width || camera_info_->height != image.height) {
      ++dropped_invalid_image_;
      throttled_warning("camera_info dimensions do not match the depth image");
      return;
    }
    if (image.is_bigendian != 0U) {
      ++dropped_invalid_image_;
      throttled_warning("big-endian depth images are not supported");
      return;
    }
    if (image.step % sizeof(float) != 0U) {
      ++dropped_invalid_image_;
      throttled_warning("depth image row step is not a whole number of float samples");
      return;
    }
    const rclcpp::Time stamp(image.header.stamp);
    if (stamp.nanoseconds() <= 0) {
      ++dropped_invalid_image_;
      throttled_warning("depth image carries no stamp");
      return;
    }
    if ((now() - stamp) > rclcpp::Duration::from_seconds(max_image_age_sec_)) {
      ++dropped_stale_image_;
      throttled_warning("depth image is older than the configured maximum age");
      return;
    }

    // Every transform below is resolved at the image's own stamp, never "the current" pose: the
    // boxes and the poses that place them must describe one instant, the exposure.
    //
    // One budget for the whole frame, in two phases (Milestone 10 §5, "One transform budget per
    // depth frame"): every lookup first runs without waiting at all -- the data is there in the
    // ordinary case -- and only if something is missing does the frame spend a single
    // `transform_timeout_sec` shared by every retry that follows. One frame used to be allowed
    // 1 + 2 * self_filter_frames sequential waits of that timeout (5.75 s nominal), and under
    // load one was measured at 10.913 s of wall time at load1 ~37 -- long enough for the
    // projector's 2.0 s obstacle window to expire while the detector sat in lookups. The budget
    // bounds how long one frame can keep the detector from the next one; a frame whose transforms
    // never arrive is still dropped, never published with a partial self-filter volume.
    const auto tf_started = std::chrono::steady_clock::now();
    Eigen::Isometry3d planning_from_optical;
    std::vector<Eigen::Isometry3d> link_at_stamp;
    std::vector<Eigen::Isometry3d> link_at_probe;
    std::string missing_frame;
    std::string failure;
    const auto probe_stamp = stamp - rclcpp::Duration::from_seconds(motion_probe_sec_);

    auto resolve_transforms = [&](std::chrono::nanoseconds budget) -> bool {
      const auto deadline = std::chrono::steady_clock::now() + budget;
      const auto timeout_left = [&deadline, budget]() -> std::chrono::nanoseconds {
        if (budget == std::chrono::nanoseconds::zero()) {
          return std::chrono::nanoseconds::zero();
        }
        const auto left = deadline - std::chrono::steady_clock::now();
        return left > std::chrono::nanoseconds::zero() ?
               left : std::chrono::nanoseconds::zero();
      };
      missing_frame.clear();
      failure.clear();
      try {
        planning_from_optical = tf2::transformToEigen(
          tf_buffer_.lookupTransform(
            planning_frame_, image.header.frame_id, tf2_ros::fromMsg(image.header.stamp),
            timeout_left()));
      } catch (const tf2::TransformException & error) {
        missing_frame = image.header.frame_id;
        failure = error.what();
        return false;
      }
      link_at_stamp.clear();
      link_at_probe.clear();
      link_at_stamp.reserve(self_filter_frames_.size());
      link_at_probe.reserve(self_filter_frames_.size());
      for (const auto & frame : self_filter_frames_) {
        try {
          link_at_stamp.push_back(
            tf2::transformToEigen(
              tf_buffer_.lookupTransform(
                planning_frame_, frame, tf2_ros::fromMsg(image.header.stamp), timeout_left())));
        } catch (const tf2::TransformException & error) {
          missing_frame = frame;
          failure = error.what();
          return false;
        }
        try {
          link_at_probe.push_back(
            tf2::transformToEigen(
              tf_buffer_.lookupTransform(
                planning_frame_, frame, tf2_ros::fromMsg(probe_stamp), timeout_left())));
        } catch (const tf2::TransformException & error) {
          missing_frame = frame;
          failure = error.what();
          return false;
        }
      }
      return true;
    };

    const bool resolved =
      resolve_transforms(std::chrono::nanoseconds::zero()) ||
      resolve_transforms(transform_timeout());
    if (!resolved) {
      // Never publish obstacles without a self-filter volume: the unfiltered link would be
      // reported as an obstacle, and an obstacle where the arm is makes every plan fail.
      ++dropped_transform_;
      record_tf_wait(tf_started);
      if (missing_frame == image.header.frame_id) {
        throttled_warning(
          std::string("sensor pose is unavailable at the image stamp: ") + failure);
      } else {
        throttled_warning(
          "self-filter frame " + missing_frame + " is unavailable at the image stamp: " + failure);
      }
      return;
    }

    std::vector<SelfFilterCapsule> capsules;
    capsules.reserve(self_filter_frames_.size());
    // How far each link's own origin moved over the probe window. A capsule endpoint would not
    // do: rail_base's endpoints are two metres out along a segment that never moves, so a rotation
    // would register there and a translation of the link would not.
    std::vector<double> probe_displacements;
    probe_displacements.reserve(self_filter_frames_.size());
    for (std::size_t index = 0; index < self_filter_frames_.size(); ++index) {
      const auto & current = link_at_stamp[index];
      // The segment is given in the link's own frame, so it rotates with the link. One lookup
      // places both ends.
      const std::size_t base = 6U * index;
      capsules.push_back(
        SelfFilterCapsule{
          current * Eigen::Vector3d(
            self_filter_segments_[base], self_filter_segments_[base + 1U],
            self_filter_segments_[base + 2U]),
          current * Eigen::Vector3d(
            self_filter_segments_[base + 3U], self_filter_segments_[base + 4U],
            self_filter_segments_[base + 5U]),
          self_filter_radii_[index] + self_filter_margin_m_});
      probe_displacements.push_back(
        (current.translation() - link_at_probe[index].translation()).norm());
    }
    record_tf_wait(tf_started);

    const bool robot_static = std::ranges::none_of(
      probe_displacements,
      [this](double displacement) {return displacement > motion_probe_tolerance_m_;});

    const std::uint32_t row_step_pixels =
      static_cast<std::uint32_t>(image.step / sizeof(float));
    std::vector<float> samples(
      static_cast<std::size_t>(row_step_pixels) * static_cast<std::size_t>(image.height));
    if (image.data.size() < samples.size() * sizeof(float)) {
      ++dropped_invalid_image_;
      throttled_warning("depth image payload is shorter than its declared dimensions");
      return;
    }
    // The ROS payload is a byte array with no alignment guarantee, so it is copied into a float
    // buffer rather than reinterpreted in place.
    std::memcpy(samples.data(), image.data.data(), samples.size() * sizeof(float));

    DepthCameraIntrinsics intrinsics;
    intrinsics.fx = camera_info_->k[0];
    intrinsics.fy = camera_info_->k[4];
    intrinsics.cx = camera_info_->k[2];
    intrinsics.cy = camera_info_->k[5];
    intrinsics.width = image.width;
    intrinsics.height = image.height;

    auto boxes = extract_obstacle_boxes(
      std::span<const float>(samples), row_step_pixels, intrinsics, planning_from_optical,
      std::span<const SelfFilterCapsule>(capsules), config_);
    if (!boxes) {
      ++dropped_extraction_;
      throttled_warning(
        std::string("obstacle extraction failed: ") + to_string(boxes.error().code) + ": " +
        boxes.error().detail);
      return;
    }

    Observation observation;
    observation.header.stamp = image.header.stamp;
    observation.header.frame_id = planning_frame_;
    observation.sensor_frame = image.header.frame_id;
    observation.sequence = ++sequence_;
    observation.robot_static = robot_static;
    observation.boxes.reserve(boxes.value().size());
    for (const auto & box : boxes.value()) {
      restocker_interfaces::msg::ObstacleBox message;
      message.center.x = box.center.x();
      message.center.y = box.center.y();
      message.center.z = box.center.z();
      message.size.x = box.size.x();
      message.size.y = box.size.y();
      message.size.z = box.size.z();
      message.point_count = static_cast<std::uint32_t>(box.point_count);
      observation.boxes.push_back(message);
    }
    observation_publisher_->publish(observation);
    ++observations_published_;
    last_publish_steady_ = std::chrono::steady_clock::now();
    last_publish_stamp_ = std::make_optional(rclcpp::Time(observation.header.stamp));
  }

  // Reports only while nothing has been published for longer than the projector's obstacle-age
  // window, so a healthy run stays silent and a stalled one names its own gap every few seconds.
  // Two clocks on purpose: the steady gap says whether this process stopped working, the stamped
  // age says what the projector (which judges the observation stamp against node time) sees. They
  // agree when the stream stopped and disagree when the clock moved instead.
  void on_heartbeat()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    const double silent_for =
      std::chrono::duration<double>(now_steady - last_publish_steady_).count();
    if (silent_for < heartbeat_silence_sec_) {
      return;
    }
    const double since_image =
      std::chrono::duration<double>(now_steady - last_image_steady_).count();
    const double stamped_age =
      last_publish_stamp_ ? (now() - *last_publish_stamp_).seconds() : -1.0;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "no obstacle observation for %.1f s wall (newest stamp %.1f s old in node time, last "
      "depth image %.1f s ago; frames received %.0f, published %.0f, dropped: no_camera_info=%.0f "
      "invalid=%.0f stale=%.0f transform=%.0f extraction=%.0f; frame processing last %.1f ms "
      "worst %.1f ms, transform wait last %.1f ms worst %.1f ms)",
      silent_for, stamped_age, since_image,
      static_cast<double>(images_received_),
      static_cast<double>(observations_published_),
      static_cast<double>(dropped_no_camera_info_),
      static_cast<double>(dropped_invalid_image_),
      static_cast<double>(dropped_stale_image_),
      static_cast<double>(dropped_transform_),
      static_cast<double>(dropped_extraction_),
      static_cast<double>(last_frame_ms_.count()),
      static_cast<double>(worst_frame_ms_.count()),
      static_cast<double>(last_tf_wait_ms_.count()),
      static_cast<double>(worst_tf_wait_ms_.count()));
  }

  void record_tf_wait(std::chrono::steady_clock::time_point started)
  {
    const auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started);
    last_tf_wait_ms_ = wait;
    if (wait > worst_tf_wait_ms_) {
      worst_tf_wait_ms_ = wait;
    }
  }

  void throttled_warning(const std::string & detail)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", detail.c_str());
  }

  std::string planning_frame_;
  double transform_timeout_sec_{0.25};
  double max_image_age_sec_{1.0};
  double heartbeat_silence_sec_{2.0};
  double motion_probe_sec_{0.20};
  double motion_probe_tolerance_m_{0.004};
  DepthObstacleConfig config_;
  std::vector<std::string> self_filter_frames_;
  std::vector<double> self_filter_segments_;
  std::vector<double> self_filter_radii_;
  double self_filter_margin_m_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Publisher<Observation>::SharedPtr observation_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::uint64_t sequence_{0U};
  // Stream accounting for the heartbeat; cumulative for the process lifetime.
  std::uint64_t images_received_{0U};
  std::uint64_t observations_published_{0U};
  std::uint64_t dropped_no_camera_info_{0U};
  std::uint64_t dropped_invalid_image_{0U};
  std::uint64_t dropped_stale_image_{0U};
  std::uint64_t dropped_transform_{0U};
  std::uint64_t dropped_extraction_{0U};
  std::chrono::steady_clock::time_point last_image_steady_{};
  std::chrono::steady_clock::time_point last_publish_steady_{};
  std::optional<rclcpp::Time> last_publish_stamp_;
  std::chrono::milliseconds last_frame_ms_{0};
  std::chrono::milliseconds worst_frame_ms_{0};
  std::chrono::milliseconds last_tf_wait_ms_{0};
  std::chrono::milliseconds worst_tf_wait_ms_{0};
};

}  // namespace restocker_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<restocker_perception::DepthObstacleNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("depth_obstacle_detector"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
