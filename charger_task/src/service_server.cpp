#include "rclcpp/rclcpp.hpp"
#include "charger_task/srv/go_to_charger.hpp"

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class ChargerServiceServer : public rclcpp::Node
{
public:
  ChargerServiceServer()
  : Node("charger_service_server")
  {
    service_ = this->create_service<charger_task::srv::GoToCharger>(
      "go_to_charger",
      std::bind(
        &ChargerServiceServer::handle_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Go To Charger service server is ready.");
  }

private:
  void handle_request(
    const std::shared_ptr<charger_task::srv::GoToCharger::Request> request,
    std::shared_ptr<charger_task::srv::GoToCharger::Response> response)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received request: distance = %.1f m",
      request->distance);

    RCLCPP_INFO(
      this->get_logger(),
      "Starting 30-second charging station trip...");

    auto start_time = this->now();

    // Simulate a long-running AGV task
    std::this_thread::sleep_for(30s);

    auto end_time = this->now();

    double travel_time = (end_time - start_time).seconds();

    response->success = true;
    response->message = "Arrived at charging station!";
    response->travel_time = travel_time;

    RCLCPP_INFO(
      this->get_logger(),
      "Trip completed. Sending response.");
  }

  rclcpp::Service<charger_task::srv::GoToCharger>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ChargerServiceServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}