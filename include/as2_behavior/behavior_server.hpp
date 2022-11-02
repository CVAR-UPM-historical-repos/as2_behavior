#ifndef AS2_NODE_TEMPLATE_HPP_
#define AS2_NODE_TEMPLATE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace as2_behavior {

template <typename actionT>
class BehaviorServer : public rclcpp::Node {
public:
  using start_srv       = typename actionT::Impl::SendGoalService;
  using modify_srv      = start_srv;
  using result_srv      = typename actionT::Impl::GetResultService;
  using feedback_msg    = typename actionT::Impl::FeedbackMessage;
  using goal_status_msg = typename actionT::Impl::GoalStatusMessage;
  using cancel_srv      = typename actionT::Impl::CancelGoalService;

  // create rclmessage formed by goal and result_msg

  typename rclcpp::Service<start_srv>::SharedPtr start_srv_;
  typename rclcpp::Service<modify_srv>::SharedPtr modify_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;

  typename rclcpp::Publisher<feedback_msg>::SharedPtr feedback_pub_;
  typename rclcpp::Publisher<goal_status_msg>::SharedPtr goal_status_pub_;

private:
  inline std::string generate_name(const std::string& name) {
    return std::string(this->get_name()) + "/" + name;
  }

public:
  BehaviorServer() : Node("behavior_server") {
    start_srv_ = this->create_service<start_srv>(
        generate_name("start"),
        std::bind(&BehaviorServer::start, this, std::placeholders::_1, std::placeholders::_2));
    modify_srv_ = this->create_service<modify_srv>(
        generate_name("modify"),
        std::bind(&BehaviorServer::modify, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        generate_name("stop"),
        std::bind(&BehaviorServer::stop, this, std::placeholders::_1, std::placeholders::_2));
    pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
        generate_name("pause"),
        std::bind(&BehaviorServer::pause, this, std::placeholders::_1, std::placeholders::_2));
    resume_srv_ = this->create_service<std_srvs::srv::Trigger>(
        generate_name("resume"),
        std::bind(&BehaviorServer::resume, this, std::placeholders::_1, std::placeholders::_2));

    feedback_pub_    = this->create_publisher<feedback_msg>(generate_name("feedback"), 10);
    goal_status_pub_ = this->create_publisher<goal_status_msg>(generate_name("goal_status"), 10);
  }

  void start(const typename start_srv::Request::SharedPtr goal,
             typename start_srv::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "start");
  };
  void stop(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
            typename std_srvs::srv::Trigger::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "stop");
  };
  void modify(const typename modify_srv::Request::SharedPtr goal,
              typename modify_srv::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "modify");
  };
  void pause(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
             typename std_srvs::srv::Trigger::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "pause");
  };
  void resume(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
              typename std_srvs::srv::Trigger::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "resume");
  };

};  // namespace as2_behavior
}  // namespace as2_behavior

#endif  // AS2_NODE_TEMPLATE_HPP_
