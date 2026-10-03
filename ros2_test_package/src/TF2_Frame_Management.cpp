#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TFGoalNode : public rclcpp::Node
{
public:
    TFGoalNode(): Node("tf_goal_node")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10,
            std::bind(&TFGoalNode::goal_callback,this,std::placeholders::_1));
            
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&TFGoalNode::broadcast_tf,this));
        RCLCPP_INFO(this->get_logger(), "TF Goal Node started");
    }

private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        map_to_odom_.transform.translation.x = msg->pose.position.x;
        map_to_odom_.transform.translation.y = msg->pose.position.y;
        map_to_odom_.transform.translation.z = msg->pose.position.z;
        map_to_odom_.transform.rotation = msg->pose.orientation;
        RCLCPP_INFO(
            this->get_logger(),
            "Received goal: x=%.2f, y=%.2f",
            msg->pose.position.x,
            msg->pose.position.y);
    }

    void broadcast_tf()
    {
        // this for map to odom
        map_to_odom_.header.stamp = this->get_clock()->now();
        map_to_odom_.header.frame_id = "map";
        map_to_odom_.child_frame_id = "odom";

        tf_broadcaster_->sendTransform(map_to_odom_);

        // this for odom to base_link - identity,so i fixed this two links together 
        geometry_msgs::msg::TransformStamped odom_to_base;
        odom_to_base.header.stamp = this->get_clock()->now();
        odom_to_base.header.frame_id = "odom";
        odom_to_base.child_frame_id = "base_link";

        odom_to_base.transform.translation.x = 0.0;
        odom_to_base.transform.translation.y = 0.0;
        odom_to_base.transform.translation.z = 0.0;
        odom_to_base.transform.rotation.x = 0.0;
        odom_to_base.transform.rotation.y = 0.0;
        odom_to_base.transform.rotation.z = 0.0;
        odom_to_base.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(odom_to_base);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped map_to_odom_;
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFGoalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

