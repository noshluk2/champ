#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class WaypointsFollower : public rclcpp::Node
{
public:
    WaypointsFollower()
    : Node("waypoints_follower")
    {
        // Declare parameters (with defaults)
        this->declare_parameter<double>("linear_speed_gain", 0.5);
        this->declare_parameter<double>("angular_speed_gain", 1.0);
        this->declare_parameter<double>("distance_tolerance", 0.05);

        // Get parameters
        linear_speed_gain_ = this->get_parameter("linear_speed_gain").as_double();
        angular_speed_gain_ = this->get_parameter("angular_speed_gain").as_double();
        distance_tolerance_ = this->get_parameter("distance_tolerance").as_double();

        // Create subscriber to PoseArray (the global path)
        path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/planner/path",
            1,
            std::bind(&WaypointsFollower::pathCallback, this, std::placeholders::_1)
        );

        // Create subscription to /odom for current robot pose
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&WaypointsFollower::odomCallback, this, std::placeholders::_1)
        );

        // Create publisher to /cmd_vel
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create a timer for the control loop (10 Hz)
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&WaypointsFollower::controlLoop, this)
        );

        // Initialize current pose
        current_x_   = 0.0;
        current_y_   = 0.0;
        current_yaw_ = 0.0;

        waypoint_index_ = 0;

        RCLCPP_INFO(this->get_logger(), "WaypointsFollower node started.");
    }

private:
    // --------------------------------------------------------------------------
    //  1) Subscriptions / Publications
    // --------------------------------------------------------------------------
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --------------------------------------------------------------------------
    //  2) Parameters
    // --------------------------------------------------------------------------
    double linear_speed_gain_;
    double angular_speed_gain_;
    double distance_tolerance_;

    // --------------------------------------------------------------------------
    //  3) Waypoints Data
    // --------------------------------------------------------------------------
    std::vector<geometry_msgs::msg::Pose> path_;
    size_t waypoint_index_; // which waypoint we are driving toward

    // --------------------------------------------------------------------------
    //  4) Current Robot Pose
    // --------------------------------------------------------------------------
    double current_x_;
    double current_y_;
    double current_yaw_;

    // --------------------------------------------------------------------------
    //  5) Callbacks
    // --------------------------------------------------------------------------
    /**
     * @brief Called whenever a new PoseArray (global path) is published.
     * We store the new set of waypoints and reset the index to 0.
     */
    void pathCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        // Copy the poses into our vector
        path_.clear();
        path_.reserve(msg->poses.size());
        for (auto &p : msg->poses)
        {
            path_.push_back(p);
        }

        waypoint_index_ = 0; // start from the first waypoint

        RCLCPP_INFO(
            this->get_logger(),
            "Received a path with %zu waypoints. Starting at waypoint_index=0.",
            path_.size()
        );
    }

    /**
     * @brief Called whenever a new Odom message arrives.
     * We update our current position and yaw.
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
     * @brief Control loop (runs at 10 Hz).
     * Moves the robot through each waypoint in path_ until completed.
     */
    void controlLoop()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;

        // If no path or we've finished all waypoints, do nothing
        if (path_.empty() || waypoint_index_ >= path_.size())
        {
            // Stop
            cmd_pub_->publish(cmd_vel);
            return;
        }

        // Get the current waypoint
        const auto &target_pose = path_[waypoint_index_];
        double goal_x = target_pose.position.x;
        double goal_y = target_pose.position.y;

        // Compute distance to current waypoint
        double dx = goal_x - current_x_;
        double dy = goal_y - current_y_;
        double distance_error = std::sqrt(dx * dx + dy * dy);

        // If we are close enough, move to the next waypoint
        if (distance_error <= distance_tolerance_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Reached waypoint %zu (%.2f, %.2f).",
                waypoint_index_, goal_x, goal_y
            );
            waypoint_index_++;
            // If we just finished the last waypoint, stop and return
            if (waypoint_index_ >= path_.size())
            {
                RCLCPP_INFO(this->get_logger(), "Path completed. Stopping.");
                cmd_pub_->publish(cmd_vel);
                return;
            }
        }
        else
        {
            // Otherwise, compute heading to the waypoint
            double heading_to_waypoint = std::atan2(dy, dx);
            double yaw_error = normalizeAngle(heading_to_waypoint - current_yaw_);

            // Simple proportional control
            cmd_vel.linear.x  = linear_speed_gain_  * distance_error;
            cmd_vel.angular.z = angular_speed_gain_ * yaw_error;

            // Optional: clamp speeds
            double max_lin = 1.2;
            double max_ang = 1.0;
            if (cmd_vel.linear.x > max_lin)  cmd_vel.linear.x = max_lin;
            if (cmd_vel.linear.x < -max_lin) cmd_vel.linear.x = -max_lin;
            if (cmd_vel.angular.z > max_ang)  cmd_vel.angular.z = max_ang;
            if (cmd_vel.angular.z < -max_ang) cmd_vel.angular.z = -max_ang;
        }

        // Publish the velocity
        cmd_pub_->publish(cmd_vel);
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
