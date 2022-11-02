#include <iostream>
#include "as2_behavior/behavior_server.hpp"

#include <rclcpp/rclcpp.hpp>
#include "as2_msgs/action/follow_path.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::cout << "Hello World!" << std::endl;
  auto node = std::make_shared<as2_behavior::BehaviorServer<as2_msgs::action::FollowPath>>();
  rclcpp::spin(node);

  return 0;
}
