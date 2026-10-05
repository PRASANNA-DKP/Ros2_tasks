#include "rclcpp/rclcpp.hpp"
#include "task_3/srv/go_to_charger.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ChargerServiceClient : public rclcpp::Node
{
public:
  ChargerServiceClient()
  : Node("charger_service_client")
  {
    client_ = this->create_client<task_3::srv::GoToCharger>(
      "go_to_charger");
  }

  void send_request()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Requesting charging station trip via SERVICE...");

    while (!client_->wait_for_service(1s)) {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for service...");
    }

    auto request =
      std::make_shared<task_3::srv::GoToCharger::Request>();

    request->distance = 60.0;

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for response (5s timeout)...");

    auto future = client_->async_send_request(request);

    auto result = rclcpp::spin_until_future_complete(
      this->get_node_base_interface(),
      future,
      5s);

    if (result == rclcpp::FutureReturnCode::SUCCESS) {

      auto response = future.get();

      RCLCPP_INFO(
        this->get_logger(),
        "Response received!");

      RCLCPP_INFO(
        this->get_logger(),
        "Success: %s",
        response->success ? "true" : "false");

      RCLCPP_INFO(
        this->get_logger(),
        "Message: %s",
        response->message.c_str());

      RCLCPP_INFO(
        this->get_logger(),
        "Travel time: %.1f s",
        response->travel_time);

    } else {

      RCLCPP_ERROR(
        this->get_logger(),
        "✘ TIMED OUT — service did not respond within 5 seconds!");
    }
  }

private:
  rclcpp::Client<task_3::srv::GoToCharger>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ChargerServiceClient>();

  node->send_request();

  rclcpp::shutdown();

  return 0;
}