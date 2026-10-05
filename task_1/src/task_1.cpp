#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"


class FrameBroadcaster : public rclcpp::Node
{
public:

    FrameBroadcaster()
    : Node("frame_broadcaster"),
      first_goal_received_(false)
    {
        // Initialize the transform broadcaster
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);


        // Subscribe for that Goal_Pose
        goal_subscription_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose",
                10,
                std::bind(
                    &FrameBroadcaster::goal_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // Publish TF continuously
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(
                &FrameBroadcaster::publish_tf,
                this
            )
        );


        RCLCPP_INFO(
            this->get_logger(),
            "Frame broadcaster started."
        );
    }


private:

    // Receive goal
    void goal_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double goal_x = msg->pose.position.x;
        double goal_y = msg->pose.position.y;

        // Convert quaternion to yaw
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;

        double goal_yaw =
            std::atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            );

        // First goal
        if (!first_goal_received_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "First goal received."
            );

            // Set fixed odom -> base_link relationship
            odom_base_x_ = 1.0;
            odom_base_y_ = 1.0;
            odom_base_yaw_ = 0.0;

            first_goal_received_ = true;
        }

        // T_map_odom = T_map_base_link * inverse(T_odom_base_link)

        double cos_yaw = std::cos(goal_yaw);
        double sin_yaw = std::sin(goal_yaw);

        map_odom_x_ =
            goal_x
            - (cos_yaw * odom_base_x_
               - sin_yaw * odom_base_y_);

        map_odom_y_ =
            goal_y
            - (sin_yaw * odom_base_x_
               + cos_yaw * odom_base_y_);

        map_odom_yaw_ =
            goal_yaw
            - odom_base_yaw_;

        RCLCPP_INFO(
            this->get_logger(),
            "Goal: x=%.2f y=%.2f yaw=%.2f",
            goal_x,
            goal_y,
            goal_yaw
        );

        RCLCPP_INFO(
            this->get_logger(),
            "map -> odom: x=%.2f y=%.2f yaw=%.2f",
            map_odom_x_,
            map_odom_y_,
            map_odom_yaw_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "odom -> base_link: x=%.2f y=%.2f",
            odom_base_x_,
            odom_base_y_
        );
    }

    // Publish TF

    void publish_tf()
    {
        rclcpp::Time current_time =
            this->get_clock()->now();

        // map -> odom
        geometry_msgs::msg::TransformStamped map_to_odom;

        map_to_odom.header.stamp = current_time;
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";


        map_to_odom.transform.translation.x = map_odom_x_;
        map_to_odom.transform.translation.y = map_odom_y_;
        map_to_odom.transform.translation.z = 0.0;


        // yaw -> quaternion
        map_to_odom.transform.rotation.x = 0.0;
        map_to_odom.transform.rotation.y = 0.0;
        map_to_odom.transform.rotation.z =std::sin(map_odom_yaw_ / 2.0);
        map_to_odom.transform.rotation.w =std::cos(map_odom_yaw_ / 2.0);

        // odom -> base_link
        geometry_msgs::msg::TransformStamped odom_to_base;

        odom_to_base.header.stamp = current_time;
        odom_to_base.header.frame_id = "odom";
        odom_to_base.child_frame_id ="base_link";

        odom_to_base.transform.translation.x = odom_base_x_;
        odom_to_base.transform.translation.y = odom_base_y_;
        odom_to_base.transform.translation.z = 0.0;

        odom_to_base.transform.rotation.x = 0.0;
        odom_to_base.transform.rotation.y =0.0;
        odom_to_base.transform.rotation.z =std::sin(odom_base_yaw_ / 2.0);
        odom_to_base.transform.rotation.w = std::cos(odom_base_yaw_ / 2.0);

        // Publish both transforms
        tf_broadcaster_->sendTransform(map_to_odom);
        tf_broadcaster_->sendTransform(odom_to_base);
    }

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster>
        tf_broadcaster_;

    // Goal subscriber

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        goal_subscription_;

    // Timer
    rclcpp::TimerBase::SharedPtr
        timer_;

    // First goal flag

    bool first_goal_received_;

    // map -> odom

    double map_odom_x_ = 0.0;
    double map_odom_y_ = 0.0;
    double map_odom_yaw_ = 0.0;

    // odom -> base_link
    // Initially 0,0
    // After first goal -> 1,1
    // Then remains fixed

    double odom_base_x_ = 0.0;
    double odom_base_y_ = 0.0;
    double odom_base_yaw_ = 0.0;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameBroadcaster>());
    rclcpp::shutdown();

    return 0;
}