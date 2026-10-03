#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "charger_task/action/go_to_charger.hpp"

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class ChargerActionServer : public rclcpp::Node
{
public:
  using GoToCharger = charger_task::action::GoToCharger;
  using GoalHandleGoToCharger =
    rclcpp_action::ServerGoalHandle<GoToCharger>;

  ChargerActionServer()
  : Node("charger_action_server")
  {
    action_server_ = rclcpp_action::create_server<GoToCharger>(
      this,
      "go_to_charger",
      std::bind(
        &ChargerActionServer::handle_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &ChargerActionServer::handle_cancel,
        this,
        std::placeholders::_1),
      std::bind(
        &ChargerActionServer::handle_accepted,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Go To Charger action server is ready.");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GoToCharger::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received action goal: distance = %.1f m",
      goal->distance);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGoToCharger> goal_handle)
  {
    (void)goal_handle;

    RCLCPP_INFO(
      this->get_logger(),
      "Received request to cancel goal.");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<GoalHandleGoToCharger> goal_handle)
  {
    // Execute the goal in a separate thread
    std::thread{
      std::bind(
        &ChargerActionServer::execute,
        this,
        std::placeholders::_1),
      goal_handle
    }.detach();
  }

  void execute(
    const std::shared_ptr<GoalHandleGoToCharger> goal_handle)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Starting 30-second charging station trip...");

    auto start_time = this->now();

    const double total_distance =
      goal_handle->get_goal()->distance;

    auto feedback =
      std::make_shared<GoToCharger::Feedback>();

    rclcpp::Rate loop_rate(1.0);

    for (int second = 1; second <= 30; ++second) {

      if (goal_handle->is_canceling()) {

        auto result =
          std::make_shared<GoToCharger::Result>();

        result->success = false;
        result->message = "Charging station trip cancelled.";
        result->travel_time =
          (this->now() - start_time).seconds();

        goal_handle->canceled(result);

        RCLCPP_INFO(
          this->get_logger(),
          "Goal cancelled.");

        return;
      }

      feedback->distance_remaining =
        total_distance * (30 - second) / 30.0;

      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(
        this->get_logger(),
        "Distance remaining: %.1f m",
        feedback->distance_remaining);

      loop_rate.sleep();
    }

    auto end_time = this->now();

    auto result =
      std::make_shared<GoToCharger::Result>();

    result->success = true;
    result->message = "Arrived at charging station!";
    result->travel_time =
      (end_time - start_time).seconds();

    goal_handle->succeed(result);

    RCLCPP_INFO(
      this->get_logger(),
      "Trip completed. Result sent.");
  }

  rclcpp_action::Server<GoToCharger>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ChargerActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}