#pragma once

#include <string>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace isaaclab
{

class PointCloudManagerNode : public rclcpp::Node
{
public:
  explicit PointCloudManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cloud1Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void cloud2Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void process();

  bool transformCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    pcl::PointCloud<pcl::PointXYZ> & out) const;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // Latest clouds
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest1_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest2_;

  // Parameters
  std::string target_frame_;
  float voxel_leaf_size_;
  float box_x_;
  float box_y_;
};

}  // namespace isaaclab
