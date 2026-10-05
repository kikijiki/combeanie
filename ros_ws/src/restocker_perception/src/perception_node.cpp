// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "restocker_perception/colour_depth_backend.hpp"
#include "restocker_perception/object_manifest.hpp"
#include "restocker_perception/observation_identity.hpp"

namespace restocker_perception
{
namespace
{

using DetectionMessage = restocker_interfaces::msg::ObjectDetection;

// A signed parameter narrowed to an unsigned count. Casting straight through would turn a
// negative value into roughly four billion, which silently suppresses every observation rather
// than reporting a bad configuration.
[[nodiscard]] std::uint32_t positive_count(std::int64_t value, const std::string & name)
{
  if (value <= 0 || value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument(name + " must be positive and fit in 32 bits");
  }
  return static_cast<std::uint32_t>(value);
}

[[nodiscard]] std::uint8_t product_class_from_name(const std::string & name)
{
  if (name == "can") {return DetectionMessage::PRODUCT_CLASS_CAN;}
  if (name == "small_bottle") {return DetectionMessage::PRODUCT_CLASS_SMALL_BOTTLE;}
  if (name == "large_bottle") {return DetectionMessage::PRODUCT_CLASS_LARGE_BOTTLE;}
  throw std::invalid_argument("'" + name + "' is not a known product class");
}

}  // namespace

// Turns the overhead RGB-D stream into normalized object observations.
//
// Running this node replaces simulator ground truth as the publisher of
// /perception/object_observations; the topic, message and admission policy are unchanged.
//
// It owns a DetectionPort and a PoseEstimationPort and knows nothing about how either works;
// replacing the classical backend with a learned one changes only its construction here.
class PerceptionNode final : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("perception"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "world");
    if (planning_frame_.empty()) {
      throw std::invalid_argument("planning_frame must not be empty");
    }
    const auto colour_topic =
      declare_parameter<std::string>("colour_topic", "/overhead_camera/image");
    const auto depth_topic =
      declare_parameter<std::string>("depth_topic", "/overhead_camera/depth_image");
    const auto camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/overhead_camera/camera_info");
    const auto observation_topic =
      declare_parameter<std::string>("observation_topic", "/perception/object_observations");
    const auto detection_topic =
      declare_parameter<std::string>("detection_topic", "/perception/detections");
    if (colour_topic.empty() || depth_topic.empty() || camera_info_topic.empty() ||
      observation_topic.empty() || detection_topic.empty())
    {
      throw std::invalid_argument("perception topics must not be empty");
    }
    const auto transform_timeout_ms =
      declare_parameter<double>("transform_timeout_ms", 60.0);
    if (!std::isfinite(transform_timeout_ms) || transform_timeout_ms < 0.0) {
      throw std::invalid_argument("transform_timeout_ms must be finite and non-negative");
    }
    transform_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(transform_timeout_ms));

    auto backend = ColourDepthBackend::create(load_backend_config());
    if (!backend) {
      throw std::invalid_argument(
              "perception backend rejected its configuration: " +
              backend.error().detail);
    }
    backend_ = std::make_unique<ColourDepthBackend>(std::move(backend.value()));
    // Card 065, diagnostic only: one INFO line per acquisition that dropped anything, naming the
    // stage and reason for every proposal. Off by default; it changes no decision.
    backend_->set_diagnostics_enabled(declare_parameter<bool>("log_refusal_diagnostics", false));

    // What is stocked, not where it is. See object_manifest.hpp.
    ObservationIdentityConfig identity;
    identity.manifest =
      load_object_manifest(declare_parameter<std::string>("object_manifest_path", ""));
    // Card 063: the gate and margin that name a category stocked more than once by pose.
    identity.association_radius_m =
      declare_parameter<double>("identity_association_radius_m", identity.association_radius_m);
    identity.ambiguity_margin_m =
      declare_parameter<double>("identity_ambiguity_margin_m", identity.ambiguity_margin_m);
    identity_ = std::make_unique<ObservationIdentityAssigner>(identity);

    observation_publisher_ = create_publisher<restocker_interfaces::msg::ObjectObservation>(
      observation_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    detection_publisher_ = create_publisher<restocker_interfaces::msg::PerceptionFrame>(
      detection_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());

    // Reliable rather than best-effort, and that is a deliberate departure from the usual
    // treatment of camera streams. Best-effort is right when a consumer only needs the latest
    // sample; here every acquisition is a pose the executor may be about to act on, and the world
    // state's own freshness gate turns a dropped one into an object that silently ages out.
    // Measured on this workcell under load, best-effort lost roughly one acquisition in ten,
    // producing gaps of over a second in a stream the executor calls stale after half of one.
    //
    // If a bridge ever publishes these best-effort, a reliable subscription is incompatible and
    // receives nothing at all, which is loud, immediate and diagnosable, unlike silent loss.
    // The parameter exists so that case can be corrected without a rebuild.
    const bool reliable_sensors = declare_parameter<bool>("reliable_sensor_qos", true);
    const auto sensor_qos = reliable_sensors ?
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable() : rclcpp::QoS(rclcpp::SensorDataQoS());
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        camera_info_ = std::move(message);
      });
    colour_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      colour_topic, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        accept(pending_colour_, std::move(message));
      });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        accept(pending_depth_, std::move(message));
      });

    RCLCPP_INFO(
      get_logger(),
      "detecting on %s and %s (%s), publishing observations on %s and detections on %s in %s",
      colour_topic.c_str(), depth_topic.c_str(),
      reliable_sensors ? "reliable" : "best effort", observation_topic.c_str(),
      detection_topic.c_str(), planning_frame_.c_str());
  }

private:
  [[nodiscard]] ColourDepthConfig load_backend_config()
  {
    ColourDepthConfig config;
    config.backend_name =
      declare_parameter<std::string>("backend_name", "overhead_rgbd_colour_depth");
    config.backend_version = declare_parameter<std::string>("backend_version", "1.0.0");
    config.chromaticity_tolerance = declare_parameter<double>("chromaticity_tolerance", 0.055);
    config.minimum_intensity = declare_parameter<double>("minimum_intensity", 60.0);
    config.minimum_depth_m = declare_parameter<double>("minimum_depth_m", 0.20);
    config.maximum_depth_m = declare_parameter<double>("maximum_depth_m", 8.0);
    config.minimum_component_pixels = positive_count(
      declare_parameter<std::int64_t>("minimum_component_pixels", 40),
      "minimum_component_pixels");
    config.top_face_band_m = declare_parameter<double>("top_face_band_m", 0.02);
    config.top_face_percentile = declare_parameter<double>("top_face_percentile", 0.92);
    config.minimum_top_face_points = positive_count(
      declare_parameter<std::int64_t>("minimum_top_face_points", 15),
      "minimum_top_face_points");
    config.minimum_mean_radius_ratio =
      declare_parameter<double>("minimum_mean_radius_ratio", 0.45);
    config.maximum_mean_radius_ratio =
      declare_parameter<double>("maximum_mean_radius_ratio", 0.95);
    config.minimum_extreme_radius_ratio =
      declare_parameter<double>("minimum_extreme_radius_ratio", 0.70);
    config.maximum_extreme_radius_ratio =
      declare_parameter<double>("maximum_extreme_radius_ratio", 1.35);
    config.height_tolerance_m = declare_parameter<double>("height_tolerance_m", 0.03);
    config.translation_sigma_floor_m =
      declare_parameter<double>("translation_sigma_floor_m", 0.003);
    // Card 063: zero keeps the top band whole (the overhead default); the wrist tray duties split
    // a same-colour column's band into its discs. See ColourDepthConfig.
    config.top_face_split_gap_m = declare_parameter<double>("top_face_split_gap_m", 0.0);

    const auto workspace_minimum =
      declare_parameter<std::vector<double>>("workspace_minimum_xyz_m", {-1.25, -1.10, 0.55});
    const auto workspace_maximum =
      declare_parameter<std::vector<double>>("workspace_maximum_xyz_m", {1.25, 1.45, 1.70});
    if (workspace_minimum.size() != 3U || workspace_maximum.size() != 3U) {
      throw std::invalid_argument("workspace bounds must each carry three values");
    }
    config.workspace.minimum =
      Eigen::Vector3d(workspace_minimum[0], workspace_minimum[1], workspace_minimum[2]);
    config.workspace.maximum =
      Eigen::Vector3d(workspace_maximum[0], workspace_maximum[1], workspace_maximum[2]);

    const auto names = declare_parameter<std::vector<std::string>>(
      "product_classes", {"can", "small_bottle", "large_bottle"});
    if (names.empty()) {
      throw std::invalid_argument("at least one product class must be configured");
    }
    for (const auto & name : names) {
      ProductSignature signature;
      signature.product_class = product_class_from_name(name);
      const auto colour =
        declare_parameter<std::vector<double>>(name + ".reference_rgb", std::vector<double>{});
      if (colour.size() != 3U) {
        throw std::invalid_argument(name + ".reference_rgb must carry three values");
      }
      signature.reference_rgb = {colour[0], colour[1], colour[2]};
      signature.sku = declare_parameter<std::string>(name + ".sku", "");
      signature.nominal_radius_m = declare_parameter<double>(name + ".nominal_radius_m", 0.0);
      signature.nominal_height_m = declare_parameter<double>(name + ".nominal_height_m", 0.0);
      config.signatures.push_back(signature);
    }
    return config;
  }

  using PendingImages = std::map<rclcpp::Time, sensor_msgs::msg::Image::ConstSharedPtr>;

  // Colour and depth are two topics off one sensor, and the executor is entitled to be told about
  // every acquisition. Holding only the latest of each loses an acquisition whenever the two
  // callbacks interleave rather than alternate, colour(t1), colour(t2), depth(t1), depth(t2)
  // leaves t1 unpaired forever even though both halves of it arrived. Under load that interleaving
  // is common, and it cost roughly one acquisition in ten. A short buffer keyed on the stamp pairs
  // them by the instant they were measured at instead of by the order they were delivered in.
  void accept(PendingImages & pending, sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    pending.emplace(rclcpp::Time(message->header.stamp, RCL_ROS_TIME), std::move(message));
    while (pending.size() > kPendingAcquisitions) {
      pending.erase(pending.begin());
    }
    drain();
  }

  void drain()
  {
    while (true) {
      auto colour = pending_colour_.begin();
      for (; colour != pending_colour_.end(); ++colour) {
        if (pending_depth_.contains(colour->first)) {
          break;
        }
      }
      if (colour == pending_colour_.end()) {
        return;
      }
      const rclcpp::Time stamp = colour->first;
      const auto depth = pending_depth_.find(stamp);
      if (!last_processed_.has_value() || stamp > *last_processed_) {
        process(colour->second, depth->second, stamp);
      }
      // Everything older than the acquisition just handled is either already processed or missing
      // its partner, and neither will improve by being kept.
      pending_colour_.erase(pending_colour_.begin(), std::next(colour));
      pending_depth_.erase(pending_depth_.begin(), std::next(depth));
    }
  }

  void process(
    const sensor_msgs::msg::Image::ConstSharedPtr & colour_,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_, const rclcpp::Time & stamp)
  {
    if (camera_info_ == nullptr) {
      return;
    }
    // The intrinsics are static on this camera, so the latest CameraInfo describes any of its
    // acquisitions. It is still checked against the image it will be applied to: a model that
    // does not describe this image is not a model of this image.
    if (camera_info_->header.frame_id != colour_->header.frame_id ||
      camera_info_->width != colour_->width || camera_info_->height != colour_->height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "camera_info does not describe the image it would be applied to");
      return;
    }
    CameraIntrinsics intrinsics;
    intrinsics.fx = camera_info_->k[0];
    intrinsics.fy = camera_info_->k[4];
    intrinsics.cx = camera_info_->k[2];
    intrinsics.cy = camera_info_->k[5];
    intrinsics.width = static_cast<std::uint16_t>(camera_info_->width);
    intrinsics.height = static_cast<std::uint16_t>(camera_info_->height);

    auto acquisition = RgbdFrame::create(intrinsics, colour_, depth_);
    if (!acquisition) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "acquisition rejected: %s",
        acquisition.error().detail.c_str());
      return;
    }

    // At the acquisition stamp, never at "latest". The overhead chain is static today, so both
    // answer the same thing; the moment this camera moves, "latest" becomes the wrong answer while
    // still looking like a transform.
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        planning_frame_, acquisition.value().frame_id(), stamp,
        tf2::durationFromSec(
          std::chrono::duration<double>(transform_timeout_).count()));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "no %s <- %s at the acquisition stamp: %s",
        planning_frame_.c_str(), acquisition.value().frame_id().c_str(), error.what());
      return;
    }
    auto camera_to_planning = FramedTransform::create(
      acquisition.value().frame_id(), planning_frame_, tf2::transformToEigen(transform));
    if (!camera_to_planning) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "extrinsics rejected: %s",
        camera_to_planning.error().detail.c_str());
      return;
    }

    last_processed_ = stamp;
    auto detections = backend_->detect(acquisition.value());
    detection_publisher_->publish(detections);
    auto observations =
      backend_->estimate(detections, acquisition.value(), camera_to_planning.value());
    if (!observations) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "pose estimation rejected the acquisition: %s",
        observations.error().detail.c_str());
      return;
    }
    // Axis-fit quality-gate refusals (Card 036): the backend has no logger, so it stashes the
    // first reason and a count. The radius/geometry gates drop the common robot-link blobs first,
    // so what reaches here is a genuine product-shaped candidate the fit could not trust.
    if (backend_->last_axis_fit_refusal_count() > 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%zu detections refused by the axis-fit quality gate: %s",
        backend_->last_axis_fit_refusal_count(), backend_->last_axis_fit_refusal().c_str());
    }
    // Card 050: the estimate gates drop proposals silently by design. When a frame's every
    // proposal was refused and nothing at all was published from it, say which gate held the
    // frame — a zero-frame dwell is then attributable from the log alone instead of being
    // indistinguishable from an empty tray. Throttled: a view containing only the robot's own
    // links produces this every frame while the arm moves.
    const auto & drops = backend_->last_estimate_drop_counts();
    if (observations.value().empty() && drops.total() > 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "every proposal in this acquisition was refused by the geometry gates: "
        "points=%u extent=%u top_face=%u radial_profile=%u axis_fit=%u nonfinite_centre=%u "
        "split_overlap=%u%s%s",
        drops.too_few_points, drops.extent, drops.top_face_points, drops.radial_profile,
        drops.axis_fit, drops.nonfinite_centre, drops.split_overlap,
        backend_->last_axis_fit_refusal_count() > 0U ? "; first axis-fit refusal: " : "",
        backend_->last_axis_fit_refusal().c_str());
    }
    if (backend_->diagnostics_enabled()) {
      log_refusal_diagnostics(stamp, observations.value().size());
    }
    const auto unidentified = identity_->assign(observations.value());
    for (const auto & binding : identity_->last_first_bindings()) {
      RCLCPP_INFO(
        get_logger(),
        "identity: first binding of '%s' at %.4f m from its declared stocking position",
        binding.source_object_id.c_str(), binding.distance_m);
    }
    if (unidentified > 0) {
      const auto & refusals = identity_->last_refusals();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%zu detections had no identity the inventory could give them and were not published "
        "(not stocked %zu, surplus %zu, outside every gate %zu, ambiguous %zu, contested %zu)%s%s",
        unidentified, refusals.not_stocked, refusals.surplus, refusals.outside_every_gate,
        refusals.ambiguous, refusals.contested, refusals.first_detail.empty() ? "" : "; first: ",
        refusals.first_detail.c_str());
    }
    for (auto & observation : observations.value()) {
      observation_publisher_->publish(observation);
    }
    RCLCPP_DEBUG(
      get_logger(), "acquisition %.3f: %zu detections, %zu observations, %zu tracks",
      stamp.seconds(), detections.detections.size(), observations.value().size(),
      identity_->track_count());
  }

  // Card 065, diagnostic only. Silent for an acquisition in which nothing was dropped.
  void log_refusal_diagnostics(const rclcpp::Time & stamp, std::size_t published) const
  {
    const auto & proposals = backend_->last_proposal_diagnostics();
    const auto & small = backend_->last_small_components();
    const bool refused = std::ranges::any_of(
      proposals, [](const ProposalDiagnostic & proposal) {
        return proposal.verdict != "published";
      });
    if (!refused && small.empty()) {
      return;
    }
    const auto & census = backend_->last_centre_census();
    std::string classified;
    for (const auto count : census.classified) {
      classified += (classified.empty() ? "" : "/") + std::to_string(count);
    }
    char buffer[256];
    std::snprintf(
      buffer, sizeof(buffer),
      " centre{px=%u depth_out=%u dark=%u no_sig=%u classified=%s rgb=(%.0f,%.0f,%.0f) "
      "depth=%.3f}",
      census.pixels, census.outside_depth_band, census.too_dark, census.no_signature,
      classified.c_str(), census.mean_red, census.mean_green, census.mean_blue,
      census.mean_depth_m);
    std::string line = buffer;
    for (const auto & proposal : proposals) {
      std::snprintf(
        buffer, sizeof(buffer),
        " {class=%u box=(%u,%u)-(%u,%u) px=%u pts=%u c=(%.3f,%.3f,%.3f) top=%.3f ext=%.3f "
        "band=%u discs=%u -> %s ",
        static_cast<unsigned>(proposal.product_class), proposal.x_min, proposal.y_min,
        proposal.x_max, proposal.y_max, proposal.mask_pixels, proposal.points,
        proposal.centroid.x(), proposal.centroid.y(), proposal.centroid.z(), proposal.top_height,
        proposal.visible_extent, proposal.band_points, proposal.discs, proposal.verdict.c_str());
      line += buffer + proposal.detail + "}";
    }
    for (const auto & component : small) {
      std::snprintf(
        buffer, sizeof(buffer), " {small class=%u px=%u box=(%u,%u)-(%u,%u)}",
        static_cast<unsigned>(component.product_class), component.pixels, component.x_min,
        component.y_min, component.x_max, component.y_max);
      line += buffer;
    }
    RCLCPP_INFO(
      get_logger(), "refusal diagnostics %.3f: %zu proposal(s), %zu published, %zu small:%s",
      stamp.seconds(), proposals.size(), published, small.size(), line.c_str());
  }

  std::string planning_frame_;
  std::chrono::nanoseconds transform_timeout_{std::chrono::milliseconds(60)};
  std::unique_ptr<ColourDepthBackend> backend_;
  std::unique_ptr<ObservationIdentityAssigner> identity_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  // Two acquisitions' worth of slack on each side, which is enough to reorder a burst without
  // holding a stale image long enough for anything to be tempted to use it.
  static constexpr std::size_t kPendingAcquisitions = 4;
  PendingImages pending_colour_;
  PendingImages pending_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  std::optional<rclcpp::Time> last_processed_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colour_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<restocker_interfaces::msg::ObjectObservation>::SharedPtr
    observation_publisher_;
  rclcpp::Publisher<restocker_interfaces::msg::PerceptionFrame>::SharedPtr detection_publisher_;
};

}  // namespace restocker_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<restocker_perception::PerceptionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("perception"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
