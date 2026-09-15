#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "a2_dual_lidar_nav/fov_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace a2_dual_lidar_nav
{

class DualLidarNavigationFilter : public rclcpp::Node
{
public:
  DualLidarNavigationFilter()
  : Node("dual_lidar_nav_filter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    const auto front_topic = declare_parameter<std::string>("front_topic", "/lidar_points");
    const auto rear_topic = declare_parameter<std::string>("rear_topic", "/lidar_points_2");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/a2/navigation/lidar_points");
    front_fov_rad_ = validatedFov("front_fov_deg", 180.0);
    rear_fov_rad_ = validatedFov("rear_fov_deg", 180.0);
    front_center_rad_ = degreesToRadians(
      declare_parameter<double>("front_center_angle_deg", 0.0));
    rear_center_rad_ = degreesToRadians(
      declare_parameter<double>("rear_center_angle_deg", 0.0));
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.20);

    if (front_topic.empty() || rear_topic.empty() || output_topic.empty() || target_frame_.empty()) {
      throw std::invalid_argument("LiDAR topics and target_frame must not be empty");
    }
    if (front_topic == rear_topic) {
      throw std::invalid_argument("front_topic and rear_topic must be different");
    }
    if (tf_timeout_sec_ < 0.0) {
      throw std::invalid_argument("tf_timeout_sec must be non-negative");
    }

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
    front_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      front_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        filterAndPublish(message, "front", front_center_rad_, front_fov_rad_);
      });
    rear_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      rear_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        filterAndPublish(message, "rear", rear_center_rad_, rear_fov_rad_);
      });

    RCLCPP_INFO(
      get_logger(),
      "Navigation LiDAR filter: front=%s rear=%s output=%s target=%s FOV=(%.1f, %.1f) deg",
      front_topic.c_str(), rear_topic.c_str(), output_topic.c_str(), target_frame_.c_str(),
      front_fov_rad_ * 180.0 / kPi, rear_fov_rad_ * 180.0 / kPi);
  }

private:
  double validatedFov(const std::string & name, double default_value)
  {
    const double degrees = declare_parameter<double>(name, default_value);
    if (!std::isfinite(degrees) || degrees <= 0.0 || degrees > 360.0) {
      throw std::invalid_argument(name + " must be in the range (0, 360]");
    }
    return degreesToRadians(degrees);
  }

  void filterAndPublish(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input,
    const char * sensor_name, double center_angle_rad, double horizontal_fov_rad)
  {
    if (!input || input->data.empty()) {
      return;
    }

    tf2::Transform transform;
    transform.setIdentity();
    if (input->header.frame_id != target_frame_) {
      try {
        const auto stamped = tf_buffer_.lookupTransform(
          target_frame_, input->header.frame_id, input->header.stamp,
          tf2::durationFromSec(tf_timeout_sec_));
        tf2::fromMsg(stamped.transform, transform);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping %s LiDAR cloud: cannot transform %s -> %s: %s",
          sensor_name, input->header.frame_id.c_str(), target_frame_.c_str(), error.what());
        return;
      }
    }

    std::vector<tf2::Vector3> accepted_points;
    accepted_points.reserve(static_cast<std::size_t>(input->width) * input->height / 2U);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*input, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*input, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*input, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*z) ||
          !withinHorizontalFov(*x, *y, center_angle_rad, horizontal_fov_rad))
        {
          continue;
        }
        const tf2::Vector3 point = transform * tf2::Vector3(*x, *y, *z);
        if (std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z())) {
          accepted_points.push_back(point);
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping %s LiDAR cloud with invalid XYZ fields: %s", sensor_name, error.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = input->header;
    output.header.frame_id = target_frame_;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(accepted_points.size());
    output.is_dense = true;

    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
    for (const auto & point : accepted_points) {
      *output_x = static_cast<float>(point.x());
      *output_y = static_cast<float>(point.y());
      *output_z = static_cast<float>(point.z());
      ++output_x;
      ++output_y;
      ++output_z;
    }
    publisher_->publish(std::move(output));
  }

  std::string target_frame_;
  double front_fov_rad_;
  double rear_fov_rad_;
  double front_center_rad_;
  double rear_center_rad_;
  double tf_timeout_sec_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rear_subscription_;
};

}  // namespace a2_dual_lidar_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<a2_dual_lidar_nav::DualLidarNavigationFilter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dual_lidar_nav_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
