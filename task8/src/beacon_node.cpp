// Copyright 2025 Locus Robotics

#include <cmath>
#include <random>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <workshop_msgs/msg/beacon_range.hpp>
#include <workshop_msgs/msg/beacon_range_array.hpp>


class BeaconNode : public rclcpp::Node
{
public:
  BeaconNode()
  : Node("beacon_node", rclcpp::NodeOptions().clock_type(RCL_ROS_TIME)),
    GRID_RESOLUTION_{15.0},
    BEACON_MAX_RANGE_SQ_{30.0 * 30.0},
    BEACON_STD_BASE_{0.01},
    gen_{dev_()},
    dist_{0.0, BEACON_STD_BASE_}
  {
    // Create beacons in a rectangular grid from (0, 0) to (250, 100)
    for (double x = 0.0; x <= 250.01; x += GRID_RESOLUTION_) {
      for (double y = 0.0; y <= 100.01; y += GRID_RESOLUTION_) {
        geometry_msgs::msg::Point beacon_point;
        beacon_point.x = x;
        beacon_point.y = y;
        beacon_point.z = 0.0;
        beacon_markers_.points.emplace_back(beacon_point);

        std_msgs::msg::ColorRGBA colour;
        colour.r = 1.0;
        colour.g = 0.0;
        colour.b = 0.0;
        colour.a = 1.0;
        beacon_markers_.colors.emplace_back(colour);
      }
    }

    beacon_markers_.action = visualization_msgs::msg::Marker::MODIFY;
    beacon_markers_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    beacon_markers_.header.frame_id = "map";
    beacon_markers_.ns = "beacons";
    beacon_markers_.id = 0;
    beacon_markers_.scale.x = 0.5;
    beacon_markers_.scale.y = 0.5;
    beacon_markers_.scale.z = 0.5;
    beacon_markers_.lifetime = rclcpp::Duration::from_nanoseconds(0);

    beacon_range_publisher_ =
      create_publisher<workshop_msgs::msg::BeaconRangeArray>("beacon_ranges", 5);
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("beacon_markers", 5);

    ground_truth_callback_ = create_subscription<nav_msgs::msg::Odometry>(
      "ground_truth", 1, std::bind(&BeaconNode::odometryCallback, this, std::placeholders::_1));
  }

private:
  void odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Poor man's throttling
    static int count = 0;

    count = (count + 1) % 100;
    if (count != 0) {
      return;
    }

    workshop_msgs::msg::BeaconRangeArray beacon_ranges;
    beacon_ranges.header = msg->header;
    beacon_markers_.header = msg->header;

    for (size_t i = 0; i < beacon_markers_.points.size(); ++i) {
      const auto & beacon_point = beacon_markers_.points[i];
      auto & beacon_colour = beacon_markers_.colors[i];

      auto x_diff = beacon_point.x - msg->pose.pose.position.x;
      auto y_diff = beacon_point.y - msg->pose.pose.position.y;
      auto dist_sq = x_diff * x_diff + y_diff * y_diff;

      if (dist_sq <= BEACON_MAX_RANGE_SQ_) {
        beacon_colour.g = 1.0;

        workshop_msgs::msg::BeaconRange beacon_range;
        beacon_range.position.x = beacon_point.x;
        beacon_range.position.y = beacon_point.y;
        beacon_range.range = std::sqrt(dist_sq);
        beacon_range.sigma = BEACON_STD_BASE_ * beacon_range.range;
        beacon_range.range += beacon_range.range * dist_(gen_);

        beacon_ranges.ranges.push_back(beacon_range);
      } else {
        beacon_colour.g = 0.0;
      }
    }

    beacon_range_publisher_->publish(beacon_ranges);
    marker_publisher_->publish(beacon_markers_);
  }

  const double GRID_RESOLUTION_;
  const double BEACON_MAX_RANGE_SQ_;
  const double BEACON_STD_BASE_;

  std::random_device dev_;
  std::mt19937 gen_;
  std::normal_distribution<double> dist_;

  visualization_msgs::msg::Marker beacon_markers_;

  rclcpp::Publisher<workshop_msgs::msg::BeaconRangeArray>::SharedPtr beacon_range_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_callback_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BeaconNode>();
  node->get_clock()->wait_until_started();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
