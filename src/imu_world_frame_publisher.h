#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace isaaclab
{

class ImuWorldFramePublisherNode : public rclcpp::Node
{
public:
  explicit ImuWorldFramePublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string imu_topic_;
  std::string world_frame_;
  std::string base_link_;
  std::string imu_frame_;

  bool has_reference_orientation_ = false;
  Eigen::Quaterniond q_w_imu_ref_ = Eigen::Quaterniond::Identity();
};

}  // namespace isaaclab
