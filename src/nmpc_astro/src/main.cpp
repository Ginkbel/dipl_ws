#include "rclcpp/rclcpp.hpp"
#include "nmpc_ros.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMPCControllerROS>());
  rclcpp::shutdown();

  return 0;
}