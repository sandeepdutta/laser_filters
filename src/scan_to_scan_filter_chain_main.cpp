#include "laser_filters/scan_to_scan_filter_chain.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto t = std::make_shared<laser_filters::ScanToScanFilterChain>(node_options);

  rclcpp::WallRate loop_rate(200);
  while (rclcpp::ok()) {

    rclcpp::spin_some(t);
    loop_rate.sleep();

  }

  return 0;
}