#ifndef AS2_BEHAVIOR_SERVER_HPP_
#define AS2_BEHAVIOR_SERVER_HPP_

#include <as2_core/node.hpp>
#include <behaviour_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>

namespace as2_behavior {

template <typename actionT>
class BehaviorServer : public as2::Node {
protected:
public:
  using GoalHandleAction = rclcpp_action::ServerGoalHandle<actionT>;
  std::string action_name_;

  typename rclcpp_action::Server<actionT>::SharedPtr action_server_;

public:
  void register_action() {
    // create a group for the action server
    auto action_server_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->action_server_ = rclcpp_action::create_server<actionT>(
        this, this->generate_global_name(action_name_),
        std::bind(&BehaviorServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&BehaviorServer::handleCancel, this, std::placeholders::_1),
        std::bind(&BehaviorServer::handleAccepted, this, std::placeholders::_1));
  };

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const typename actionT::Goal> goal) {
    RCLCPP_DEBUG(this->get_logger(), "Received goal request with UUID: %s", (char*)uuid.data());
    if (this->activate(goal)) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Request to cancel goal received");
    std_srvs::srv::Trigger::Request::SharedPtr req =
        std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response::SharedPtr res =
        std::make_shared<std_srvs::srv::Trigger::Response>();
    deactivate(req, res);
    if (res->success) {
      return rclcpp_action::CancelResponse::ACCEPT;
    } else {
      return rclcpp_action::CancelResponse::REJECT;
    }
  };

  void handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle) {
    goal_handle_ = goal_handle;
  };
  std::shared_ptr<GoalHandleAction> goal_handle_;

  as2_msgs::msg::BehaviorStatus behavior_status_;
  using BehaviorStatus  = as2_msgs::msg::BehaviorStatus;
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

  rclcpp::Publisher<BehaviorStatus>::SharedPtr behavior_status_pub_;
  rclcpp::TimerBase::SharedPtr behavior_status_timer_;
  rclcpp::TimerBase::SharedPtr run_timer_;

private:
  std::string generate_name(const std::string& name) {
    return std::string(this->get_name()) + "_behavior/" + name;
  }

private:
  void register_service_servers() {
    /* start_srv_ = this->create_service<start_srv>(
        generate_name("activate"),
        std::bind(&BehaviorServer::activate, this, std::placeholders::_1, std::placeholders::_2));
    modify_srv_ = this->create_service<modify_srv>(
        generate_name("modify"),
        std::bind(&BehaviorServer::modify, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        generate_name("deactivate"),
        std::bind(&BehaviorServer::deactivate, this, std::placeholders::_1,
    std::placeholders::_2));*/
    pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
        generate_name("pause"),
        std::bind(&BehaviorServer::pause, this, std::placeholders::_1, std::placeholders::_2));
    resume_srv_ = this->create_service<std_srvs::srv::Trigger>(
        generate_name("resume"),
        std::bind(&BehaviorServer::resume, this, std::placeholders::_1, std::placeholders::_2));
  }

  void register_publishers() {
    feedback_pub_    = this->create_publisher<feedback_msg>(generate_name("feedback"), 10);
    goal_status_pub_ = this->create_publisher<goal_status_msg>(generate_name("goal_status"), 10);
    behavior_status_pub_ =
        this->create_publisher<BehaviorStatus>(generate_name("behavior_status"), 10);
  }
  void register_timers() {
    behavior_status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&BehaviorServer::publish_behavior_status, this));
  }

  template <typename T = long, typename ratio_ = std::ratio<1, 1000>>
  void register_run_timer(
      std::chrono::duration<T, ratio_> period = std::chrono::milliseconds(100)) {
    run_timer_ = this->create_wall_timer(period, std::bind(&BehaviorServer::run, this));
  }
  void cleanup_run_timer() {
    goal_handle_.reset();
    run_timer_.reset();
  }

public:
  BehaviorServer(const std::string& name) : as2::Node(name), action_name_(name) {
    register_action();
    register_service_servers();
    register_publishers();
    register_timers();
  }

  // TODO: CONVERT INTO PURE VIRTUAL FUNCTIONS
  virtual bool on_activate(std::shared_ptr<const typename actionT::Goal> goal) { return true; };
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
    result->set__success(true);
    return true;
  }
  virtual bool on_resume(const std_srvs::srv::Trigger::Request::SharedPtr goal,
                         std_srvs::srv::Trigger::Response::SharedPtr result) {
    result->set__success(true);
    return true;
  }

  virtual ExecutionState on_run(typename feedback_msg::SharedPtr& fb) {
    return ExecutionState::SUCCESS;
  }

  bool activate(std::shared_ptr<const typename actionT::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "start");
    if (on_activate(goal)) {
      register_run_timer();
      behavior_status_.status = BehaviorStatus::RUNNING;
      return true;
    }
    return false;
  };
  void deactivate(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
                  typename std_srvs::srv::Trigger::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "stop");
    on_deactivate(goal, result);
    cleanup_run_timer();
  };
  void modify(const typename modify_srv::Request::SharedPtr goal,
              typename modify_srv::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "modify");
    on_modify(goal, result);
  };
  void pause(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
             typename std_srvs::srv::Trigger::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "pause");
    if (behavior_status_.status != BehaviorStatus::RUNNING) {
      result->success = false;
      result->message = "Behavior is not running";
      return;
    }
    on_pause(goal, result);
    if (result->success) {
      behavior_status_.status = BehaviorStatus::PAUSED;
    }
  };
  void resume(const typename std_srvs::srv::Trigger::Request::SharedPtr goal,
              typename std_srvs::srv::Trigger::Response::SharedPtr result) {
    RCLCPP_INFO(this->get_logger(), "resume");
    if (behavior_status_.status != BehaviorStatus::PAUSED) {
      result->success = false;
      result->message = "Behavior is not paused";
      return;
    }
    on_resume(goal, result);
    if (result->success) {
      behavior_status_.status = BehaviorStatus::RUNNING;
    }
  };

  void run() {
    if (behavior_status_.status != BehaviorStatus::RUNNING) {
      return;
    };
    auto fb              = std::make_shared<feedback_msg>();
    ExecutionState state = on_run(fb);
    // feedback_pub_->publish(fb);

    switch (state) {
      case ExecutionState::SUCCESS:
        RCLCPP_INFO(this->get_logger(), "SUCCESS");
        behavior_status_.status = BehaviorStatus::IDLE;
        // goal_handle_->succeed();
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

    if (behavior_status_.status != BehaviorStatus::RUNNING) {
      // TODO: CHECK IF THIS IS NEEDED
      cleanup_run_timer();
    }
  }

  void publish_behavior_status() {
    BehaviorStatus msg;
    msg.status = behavior_status_.status;
    behavior_status_pub_->publish(msg);
  }
};  // namespace as2_behavior
};  // namespace as2_behavior

#endif  // AS2_NODE_TEMPLATE_HPP_
