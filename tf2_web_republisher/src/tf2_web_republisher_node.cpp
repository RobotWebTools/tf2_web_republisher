#include <rclcpp/rclcpp.hpp>
#include "tf2_web_republisher/tf2_web_republisher.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto tf2_web_republisher = std::make_shared<TFRepublisher>("tf2_web_republisher");

  rclcpp::spin(tf2_web_republisher);

  return 0;
}
