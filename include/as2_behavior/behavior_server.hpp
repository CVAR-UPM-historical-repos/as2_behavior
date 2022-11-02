#ifndef AS2_BEHAVIOR_SERVER_HPP_
#define AS2_BEHAVIOR_SERVER_HPP_

#include <behaviour_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

namespace as2_behavior {

template <typename actionT>
class BehaviorServer : public rclcpp::Node {
protected:
  as2_behavior::msg::BehaviorStatus behavior_status_;
  using BehaviorStatus  = as2_behavior::msg::BehaviorStatus;
  using start_srv       = typename actionT::Impl::SendGoalService;
  using modify_srv      = start_srv;
  using result_srv      = typename actionT::Impl::GetResultService;
  using feedback_msg    = typename actionT::Impl::FeedbackMessage;
  using goal_status_msg = typename actionT::Impl::GoalStatusMessage;
  using cancel_srv      = typename actionT::Impl::CancelGoalService;

private:
  // create rclmessage formed by goal and result_msg
  typename rclcpp::Service<start_srv>::SharedPtr start_srv_;
  typename rclcpp::Service<modify_srv>::SharedPtr modify_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;

  typename rclcpp::Publisher<feedback_msg>::SharedPtr feedback_pub_;
  typename rclcpp::Publisher<goal_status_msg>::SharedPtr goal_status_pub_;

  rclcpp::Publisher<as2_behavior::msg::BehaviorStatus>::SharedPtr behavior_status_pub_;
  rclcpp::TimerBase::SharedPtr behavior_status_timer_;

private:
  std::string generate_name(const std::string& name) {
    return std::string(this->get_name()) + "_behavior/" + name;
  }

public:
  BehaviorServer(const std::string& name) : Node(name) {
    start_srv_ = this->create_service<start_srv>(
        generate_name("activate"),
        std::bind(&BehaviorServer::activate, this, std::placeholders::_1, std::placeholders::_2));
    modify_srv_ = this->create_service<modify_srv>(
        generate_name("modify"),
        std::bind(&BehaviorServer::modify, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        generate_name("deactivate"),
        std::bind(&BehaviorServer::deactivate, this, std::placeholders::_1, std::placeholders::_2));
    pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
        generate_name("pause"),
        std::bind(&BehaviorServer::pause, this, std::placeholders::_1, std::placeholders::_2));
    resume_srv_ = this->create_service<std_srvs::srv::Trigger>(
        generate_name("resume"),
        std::bind(&BehaviorServer::resume, this, std::placeholders::_1, std::placeholders::_2));
    feedback_pub_    = this->create_publisher<feedback_msg>(generate_name("feedback"), 10);
    goal_status_pub_ = this->create_publisher<goal_status_msg>(generate_name("goal_status"), 10);

    behavior_status_pub_ = this->create_publisher<as2_behavior::msg::BehaviorStatus>(
        generate_name("behavior_status"), 10);
    behavior_status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&BehaviorServer::publish_behavior_status, this));
  }

  // TODO: CONVERT INTO PURE VIRTUAL FUNCTIONS
  virtual bool on_activate(const typename start_srv::Request::SharedPtr goal,
                           typename start_srv::Response::SharedPtr result) {
    return true;
  }
  virtual bool on_modify(const typename modify_srv::Request::SharedPtr goal,
                         typename modify_srv::Response::SharedPtr result) {
    return true;
  }
  virtual bool on_deactivate(const std_srvs::srv::Trigger::Request::SharedPtr goal,
                             std_srvs::srv::Trigger::Response::SharedPtr result) {
    return true;
  }
  virtual bool on_pause(const std_srvs::srv::Trigger::Request::SharedPtr goal,
                        std_srvs::srv::Trigger::Response::SharedPtr result) {
    return true;
  }
  virtual bool on_resume(const std_srvs::srv::Trigger::Request::SharedPtr goal,
                         std_srvs::srv::Trigger::Response::SharedPtr result) {
    return true;
  }
  virtual ExecutionState on_run() { return ExecutionState::SUCCESS; }

  void activate(const typename start_srv::Request::SharedPtr goal,
                typename start_srv::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "start");
    if (on_activate(goal, result)) {
      behavior_status_.status = BehaviorStatus::RUNNING;
    }
  };
  void deactivate(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
                  typename std_srvs::srv::Trigger::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "stop");
    on_deactivate(goal, result);
  };
  void modify(const typename modify_srv::Request::SharedPtr goal,
              typename modify_srv::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "modify");
    on_modify(goal, result);
  };
  void pause(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
             typename std_srvs::srv::Trigger::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "pause");
    on_pause(goal, result);
  };
  void resume(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
              typename std_srvs::srv::Trigger::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "resume");
    on_resume(goal, result);
  };

  void run() {
    if (behavior_status_.status != BehaviorStatus::RUNNING) {
      return;
    };
    ExecutionState state = on_run();
    switch (state) {
      case ExecutionState::SUCCESS:
        RCLCPP_INFO(this->get_logger(), "SUCCESS");
        behavior_status_.status = BehaviorStatus::IDLE;
        break;
      case ExecutionState::RUNNING:
        RCLCPP_INFO(this->get_logger(), "RUNNING");
        behavior_status_.status = BehaviorStatus::RUNNING;
        break;
      case ExecutionState::FAILURE:
        RCLCPP_INFO(this->get_logger(), "FAILURE");
        behavior_status_.status = BehaviorStatus::IDLE;
        break;
      case ExecutionState::ABORTED:
        RCLCPP_INFO(this->get_logger(), "ABORTED");
        behavior_status_.status = BehaviorStatus::IDLE;
        break;
    }
  }

  void publish_behavior_status() {
    as2_behavior::msg::BehaviorStatus msg;
    msg.status = behavior_status_.status;
    behavior_status_pub_->publish(msg);
  }
};

};  // namespace as2_behavior

#endif  // AS2_NODE_TEMPLATE_HPP_
