#ifndef PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/buffer.h"

namespace pure_pursuit_controller
{

class PurePursuitController : public nav2_core::Controller
{
public:

  PurePursuitController() = default;

  ~PurePursuitController() override = default;

  // ---------------------------------------------------------
  // Nav2 Controller lifecycle methods
  // ---------------------------------------------------------

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  // ---------------------------------------------------------
  // Receive the global path from Nav2 Planner Server
  // ---------------------------------------------------------

  void setPlan(
    const nav_msgs::msg::Path & path) override;

  // ---------------------------------------------------------
  // Calculate velocity command
  // ---------------------------------------------------------

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  // ---------------------------------------------------------
  // Speed limit interface required by Nav2
  // ---------------------------------------------------------

  void setSpeedLimit(
    const double & speed_limit,
    const bool & percentage) override;

private:

  // ---------------------------------------------------------
  // Nav2 node and TF
  // ---------------------------------------------------------

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string plugin_name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // ---------------------------------------------------------
  // Global path received from Nav2
  // ---------------------------------------------------------

  nav_msgs::msg::Path global_plan_;

  // ---------------------------------------------------------
  // Controller parameters
  // ---------------------------------------------------------

  double linear_velocity_ = 0.2;

  double angular_gain_ = 1.0;

  // ---------------------------------------------------------
  // Path waypoint tracking
  // ---------------------------------------------------------

  size_t current_waypoint_index_ = 0;

  double waypoint_tolerance_ = 0.15;
};

}  // namespace pure_pursuit_controller

#endif