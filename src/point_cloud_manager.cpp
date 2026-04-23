#include "point_cloud_manager.h"

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

namespace isaaclab
{

PointCloudManagerNode::PointCloudManagerNode(const rclcpp::NodeOptions & options)
: Node("point_cloud_manager", options)
{
  declare_parameter<std::string>("topic1", "/cloud1");
  declare_parameter<std::string>("topic2", "/cloud2");
  declare_parameter<std::string>("output_topic", "/cloud_merged");
  declare_parameter<std::string>("target_frame", "base_link");
  declare_parameter<float>("voxel_leaf_size", 0.05f);
  declare_parameter<float>("box_x", 2.0f);
  declare_parameter<float>("box_y", 1.5f);

  const auto topic1     = get_parameter("topic1").as_string();
  const auto topic2     = get_parameter("topic2").as_string();
  const auto out_topic  = get_parameter("output_topic").as_string();
  target_frame_         = get_parameter("target_frame").as_string();
  voxel_leaf_size_      = static_cast<float>(get_parameter("voxel_leaf_size").as_double());
  box_x_                = static_cast<float>(get_parameter("box_x").as_double());
  box_y_                = static_cast<float>(get_parameter("box_y").as_double());

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::SensorDataQoS();

  sub1_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    topic1, qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      latest1_ = msg;
      process();
    });

  sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    topic2, qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      latest2_ = msg;
      process();
    });

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "PointCloudManagerNode ready.");
  RCLCPP_INFO(get_logger(), "  topic1: %s", topic1.c_str());
  RCLCPP_INFO(get_logger(), "  topic2: %s", topic2.c_str());
  RCLCPP_INFO(get_logger(), "  output: %s", out_topic.c_str());
  RCLCPP_INFO(get_logger(), "  target_frame: %s", target_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  voxel_leaf: %.3f m", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "  box_x: %.2f m  box_y: %.2f m", box_x_, box_y_);
}

bool PointCloudManagerNode::transformCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  pcl::PointCloud<pcl::PointXYZ> & out) const
{
  sensor_msgs::msg::PointCloud2 transformed_msg;
  try {
    // Use latest available transform; robust against sim-time jumps
    const auto tf = tf_buffer_->lookupTransform(
      target_frame_, msg->header.frame_id,
      rclcpp::Time(0, 0, get_clock()->get_clock_type()),
      rclcpp::Duration(std::chrono::milliseconds(100)));
    tf2::doTransform(*msg, transformed_msg, tf);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Transform to '%s' failed: %s", target_frame_.c_str(), ex.what());
    return false;
  }
  pcl::fromROSMsg(transformed_msg, out);
  return true;
}

void PointCloudManagerNode::process()
{
  if (!latest1_ || !latest2_) {
    return;
  }

  // --- Transform both clouds to target_frame ---
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  if (!transformCloud(latest1_, cloud1) || !transformCloud(latest2_, cloud2)) {
    return;
  }

  // --- Merge ---
  pcl::PointCloud<pcl::PointXYZ> merged;
  merged = cloud1;
  merged += cloud2;

  // --- Box filter: keep points inside [-box_x/2, box_x/2] x [-box_y/2, box_y/2] ---
  {
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(merged.makeShared());
    crop.setMin(Eigen::Vector4f(-box_x_ / 2.0f, -box_y_ / 2.0f, -std::numeric_limits<float>::max(), 1.0f));
    crop.setMax(Eigen::Vector4f( box_x_ / 2.0f,  box_y_ / 2.0f,  std::numeric_limits<float>::max(), 1.0f));
    crop.filter(merged);
  }

  // --- Voxel downsampling ---
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(merged.makeShared());
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(merged);
  }

  // --- Publish ---
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(merged, out_msg);
  out_msg.header.frame_id = target_frame_;
  out_msg.header.stamp    = get_clock()->now();
  pub_->publish(out_msg);
}

}  // namespace isaaclab

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<isaaclab::PointCloudManagerNode>());
  rclcpp::shutdown();
  return 0;
}
