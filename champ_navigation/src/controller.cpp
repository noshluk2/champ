/**
 * @file go_to_goal_improved.cpp
 * @brief A ROS 2 node for improved go-to-goal control that avoids circling.
 *
 * This node separates the heading control from forward motion when far from the goal:
 *  1. Rotate in place if yaw error is large (above heading_threshold).
 *  2. Drive toward the goal if yaw error is small enough.
 *  3. Once within distance_tolerance, stop forward motion and rotate to reach goal_yaw.
 *
 * Tweak gains, thresholds, and tolerances as needed.
 */

#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class GoToGoalController : public rclcpp::Node
{
public:
    GoToGoalController()
    : Node("go_to_goal_controller")
    {
        // Declare parameters (with defaults)
        this->declare_parameter("goal_x", 1.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("goal_yaw", 0.0);
        this->declare_parameter("linear_speed_gain", 0.5);
        this->declare_parameter("angular_speed_gain", 1.0);
        this->declare_parameter("distance_tolerance", 0.05);
        this->declare_parameter("yaw_tolerance", 0.05);

        // Additional parameter to decide when to rotate in place vs. move
        this->declare_parameter("heading_threshold", 0.3); // rad

        // Get parameters
        goal_x_            = this->get_parameter("goal_x").as_double();
        goal_y_            = this->get_parameter("goal_y").as_double();
        goal_yaw_          = this->get_parameter("goal_yaw").as_double();
        linear_speed_gain_ = this->get_parameter("linear_speed_gain").as_double();
        angular_speed_gain_= this->get_parameter("angular_speed_gain").as_double();
        distance_tolerance_= this->get_parameter("distance_tolerance").as_double();
        yaw_tolerance_     = this->get_parameter("yaw_tolerance").as_double();
        heading_threshold_ = this->get_parameter("heading_threshold").as_double();

        // Create subscription to /odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&GoToGoalController::odomCallback, this, std::placeholders::_1)
        );

        // Create publisher to /cmd_vel
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Create timer for control loop (10 Hz)
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&GoToGoalController::controlLoop, this)
        );

        // Initialize pose
        current_x_   = 0.0;
        current_y_   = 0.0;
        current_yaw_ = 0.0;
    }

private:
    /**
     * @brief Timer callback implementing the improved go-to-goal logic.
     */
    void controlLoop()
    {
        // Calculate distance to goal
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double distance_error = std::sqrt(dx * dx + dy * dy);

        // Calculate heading to goal and yaw error
        double heading_to_goal = std::atan2(dy, dx);
        double yaw_error = normalizeAngle(heading_to_goal - current_yaw_);

        // Prepare Twist message
        geometry_msgs::msg::Twist cmd_vel;

        // 1. Check if we've reached the goal position
        if (distance_error <= distance_tolerance_)
        {
            // Already close to goal's position. Now check orientation.
            double yaw_goal_error = normalizeAngle(goal_yaw_ - current_yaw_);

            if (std::fabs(yaw_goal_error) > yaw_tolerance_)
            {
                // Rotate in place to match the final yaw
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = angular_speed_gain_ * yaw_goal_error;
            }
            else
            {
                // Goal fully reached (position + orientation)
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = 0.0;
            }
        }
        else
        {
            // 2. Not yet at goal position
            // Decide whether to rotate in place or move forward
            if (std::fabs(yaw_error) > heading_threshold_)
            {
                // Large yaw error => rotate in place first
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = angular_speed_gain_ * yaw_error;
            }
            else
            {
                // Small yaw error => move forward + small turn
                cmd_vel.linear.x  = linear_speed_gain_  * distance_error;
                cmd_vel.angular.z = angular_speed_gain_ * yaw_error;
            }
        }

        // Optional: limit speeds
        const double max_linear_speed  = 0.3; // m/s
        const double max_angular_speed = 1.0; // rad/s

        if (cmd_vel.linear.x > max_linear_speed)
            cmd_vel.linear.x = max_linear_speed;
        if (cmd_vel.linear.x < -max_linear_speed)
            cmd_vel.linear.x = -max_linear_speed;
        if (cmd_vel.angular.z > max_angular_speed)
            cmd_vel.angular.z = max_angular_speed;
        if (cmd_vel.angular.z < -max_angular_speed)
            cmd_vel.angular.z = -max_angular_speed;

        // Publish velocity command
        cmd_pub_->publish(cmd_vel);
    }

    /**
     * @brief Callback to update current pose from odometry.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // Convert quaternion to yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        current_yaw_ = yaw;
    }

    /**
     * @brief Normalize an angle to [-π, π].
     */
    double normalizeAngle(double angle)
    {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

private:
    // Subscriptions and Publications
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double goal_x_;
    double goal_y_;
    double goal_yaw_;
    double linear_speed_gain_;
    double angular_speed_gain_;
    double distance_tolerance_;
    double yaw_tolerance_;

    // Additional heading threshold to decide rotate-in-place vs. move forward
    double heading_threshold_;

    // Current pose
    double current_x_;
    double current_y_;
    double current_yaw_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToGoalController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
