#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "height_scan_builder.h"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<HeightScanBuilder>();
	rclcpp::WallRate rate(node->loopRateHz());

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
		node->buildHeightScan();
		rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}