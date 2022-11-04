#include <iostream>
#include "as2_behavior/behavior_server.hpp"

#include <rclcpp/rclcpp.hpp>
// #include "as2_msgs/action/follow_path.hpp"
#include "as2_msgs/action/take_off.hpp"

class TakeOffServer : public as2_behavior::BehaviorServer<as2_msgs::action::TakeOff> {
public:
  TakeOffServer(const std::string& name)
      : as2_behavior::BehaviorServer<as2_msgs::action::TakeOff>(name) {
    std::cout << "TakeOffServer constructor" << std::endl;
  }
  as2_behavior::ExecutionState on_run(typename feedback_msg::SharedPtr& fb) override {
    return as2_behavior::ExecutionState::RUNNING;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::cout << "Hello World!" << std::endl;
  auto node = std::make_shared<TakeOffServer>("TakeOffBehaviour");
  rclcpp::spin(node);

  return 0;
}
