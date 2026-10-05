#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_3/action/go_to_charger.hpp"

#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

class ChargerActionClient : public rclcpp::Node
{
public:
  using GoToCharger = task_3::action::GoToCharger;
  using GoalHandleGoToCharger =
    rclcpp_action::ClientGoalHandle<GoToCharger>;

  ChargerActionClient()
  : Node("charger_action_client")
  {
    client_ =
      rclcpp_action::create_client<GoToCharger>(
        this,
        "go_to_charger");
  }

  void send_goal()
  {
    if (!client_->wait_for_action_server(5s)) {

      RCLCPP_ERROR(
        this->get_logger(),
        "Action server not available.");

      rclcpp::shutdown();
      return;
    }

    GoToCharger::Goal goal_msg;

    goal_msg.distance = 60.0;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending Go To Charger action goal...");

    RCLCPP_INFO(
      this->get_logger(),
      "Goal accepted — receiving feedback...");

    rclcpp_action::Client<GoToCharger>::SendGoalOptions options;

    options.goal_response_callback =
      std::bind(
        &ChargerActionClient::goal_response_callback,
        this,
        std::placeholders::_1);

    options.feedback_callback =
      std::bind(
        &ChargerActionClient::feedback_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2);

    options.result_callback =
      std::bind(
        &ChargerActionClient::result_callback,
        this,
        std::placeholders::_1);

    client_->async_send_goal(goal_msg, options);
  }

private:
  void goal_response_callback(
    const GoalHandleGoToCharger::SharedPtr & goal_handle)
  {
    if (!goal_handle) {

      RCLCPP_ERROR(
        this->get_logger(),
        "Goal was rejected by server.");

    } else {

      RCLCPP_INFO(
        this->get_logger(),
        "Goal accepted by server.");
    }
  }

  void feedback_callback(
    GoalHandleGoToCharger::SharedPtr,
    const std::shared_ptr<const GoToCharger::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[feedback] Distance remaining: %.1f m",
      feedback->distance_remaining);
  }

  void result_callback(
    const GoalHandleGoToCharger::WrappedResult & result)
  {
    if (result.code ==
        rclcpp_action::ResultCode::SUCCEEDED) {

      RCLCPP_INFO(
        this->get_logger(),
        "✔ Result: %s Travel time: %.1fs",
        result.result->message.c_str(),
        result.result->travel_time);

    } else {

      RCLCPP_ERROR(
        this->get_logger(),
        "Action did not succeed.");
    }

    rclcpp::shutdown();
  }

  rclcpp_action::Client<GoToCharger>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ChargerActionClient>();

  node->send_goal();

  rclcpp::spin(node);

  return 0;
}