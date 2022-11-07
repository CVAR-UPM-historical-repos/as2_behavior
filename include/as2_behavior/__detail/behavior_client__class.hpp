#ifndef __BEHAVIOR_CLIENT__CLASS_HPP__
#define __BEHAVIOR_CLIENT__CLASS_HPP__

#include <as2_behavior/behaviour_utils.hpp>
#include <as2_core/node.hpp>
#include <as2_core/synchronous_service_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>

namespace as2_behavior {

/*

{
public:
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
: Node("fibonacci_action_client", options)
{
  this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
    this,
    "fibonacci");

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&FibonacciActionClient::send_goal, this));
}

void send_goal()
{
  using namespace std::placeholders;

  this->timer_->cancel();

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 10;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&FibonacciActionClient::result_callback, this, _1);
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

private:
rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
rclcpp::TimerBase::SharedPtr timer_;

void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void feedback_callback(
  GoalHandleFibonacci::SharedPtr,
  const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
  std::stringstream ss;
  ss << "Next number in sequence received: ";
  for (auto number : feedback->partial_sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void result_callback(const GoalHandleFibonacci::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  std::stringstream ss;
  ss << "Result received: ";
  for (auto number : result.result->sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  rclcpp::shutdown();
}
 * */

template <typename actionT>
class BehaviorClient : public as2::Node {
protected:
public:
  using GoalHandleAction = rclcpp_action::ServerGoalHandle<actionT>;
  std::string action_name_;
  typename rclcpp_action::Client<actionT>::SharedPtr action_client_;

public:
  typename rclcpp_action::Client<actionT>::SendGoalOptions send_goal_options_ =
      rclcpp_action::Client<actionT>::SendGoalOptions();

  void register_action();
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const typename actionT::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleAction> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle);

  as2_msgs::msg::BehaviorStatus behavior_status_;

  using BehaviorStatus = as2_msgs::msg::BehaviorStatus;

private:
  as2::SynchronousServiceClient<std_srvs::srv::Trigger> pause_srv_;
  as2::SynchronousServiceClient<std_srvs::srv::Trigger> resume_srv_;

  typename rclcpp::Subscription<typename actionT::Feedback>::SharedPtr feedback_sub_;
  typename rclcpp::Subscription<as2_msgs::msg::BehaviorStatus>::SharedPtr status_sub_;

private:
  std::string generate_name(const std::string& name);

private:
  void register_publishers();
  void register_timers();

  void register_run_timer();
  void cleanup_run_timer(const ExecutionStatus& status);

  rclcpp::Node* node_ptr_;

public:
  BehaviorClient(rclcpp::Node* node_ptr, const std::string& name)
      : node_ptr_(node_ptr), action_name_(name), pause_srv_(node_ptr_, generate_name("pause")),
        resume_srv_(node_ptr_, generate_name("resume")) {
    this->action_client_ = rclcpp_action::create_client<actionT>(this, name);
  };

  void goal_response_callback(std::shared_future<typename GoalHandleAction::SharedPtr> future);

  bool activate(std::shared_ptr<const typename actionT::Goal> goal,
                std::function<void()> fb_callback) {
    send_goal_options_.goal_response_callback =
        std::bind(&BehaviorClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback =
        std::bind(fb_callback, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback = std::bind(result_callback, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
};  // namespace as2_behavior
bool modify(std::shared_ptr<const typename actionT::Goal> goal){};
bool deactivate(const std::shared_ptr<std::string>& message) {
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
  auto out = pause_srv_.sendRequest(req, res);
  *message = res->message;
  return out && res->success;
};
bool resume(const std::shared_ptr<std::string>& message) {
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
  auto out = resume_srv_.sendRequest(req, res);
  *message = res->message;
  return out && res->success;
};
bool pause(const std::shared_ptr<std::string>& message) {
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
  auto out = pause_srv_.sendRequest(req, res);
  *message = res->message;
  return out && res->success;
};
as2_msgs::msg::BehaviorStatus get_status();
};  // namespace as2_behavior
}
;  // namespace as2_behavior

#endif  // __BEHAVIOR_SERVER__CLASS_HPP__

