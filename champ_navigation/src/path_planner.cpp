#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

// PCL + conversions
#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/PlannerStatus.h>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // doTransform

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner3D_FixedZ_SubOdom : public rclcpp::Node
{
public:
    Planner3D_FixedZ_SubOdom()
      : Node("planner_3d_fixed_z_sub_odom"),
        octree_resolution_(0.1f),
        octree_(octree_resolution_)
    {
        // TF buffer + listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to the filtered cloud
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/filtered_point_cloud", 10,
            std::bind(&Planner3D_FixedZ_SubOdom::cloudCallback, this, std::placeholders::_1));

        // Subscribe to a single PoseStamped goal
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1,
            std::bind(&Planner3D_FixedZ_SubOdom::goalCallback, this, std::placeholders::_1));

        // Subscribe to /odom to get the robot’s current pose
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&Planner3D_FixedZ_SubOdom::odomCallback, this, std::placeholders::_1));

        // Publisher for the final path
        path_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/planner/path", 10);

        // Initialize the 3D planner (fixed z)
        initializePlanner();

        RCLCPP_INFO(get_logger(), "Planner3D_FixedZ_SubOdom node started.");
    }

private:
    // --------------------------------------------------------------------------
    //  1) Octree for collision-check
    // --------------------------------------------------------------------------
    float octree_resolution_;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_;
    std::mutex octree_mutex_;

    // --------------------------------------------------------------------------
    //  2) Subscriptions / Publications
    // --------------------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;

    // --------------------------------------------------------------------------
    //  3) TF2 Buffer + Listener
    // --------------------------------------------------------------------------
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // --------------------------------------------------------------------------
    //  4) OMPL: 3D space, but fix z=0.5
    // --------------------------------------------------------------------------
    og::SimpleSetupPtr simple_setup_;

    // --------------------------------------------------------------------------
    //  5) Current Robot Pose (from /odom)
    // --------------------------------------------------------------------------
    std::mutex pose_mutex_;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0; // you might not strictly need yaw for 3D planning, but let's store it

    // For a fixed z plan at 0.5, we might not rely on odom’s z. But if you need it, store it here:
    // double current_z_ = 0.0;

    void initializePlanner()
    {
        // We'll plan in 3D: (x, y, z) but fix z in [0.5, 0.5]
        auto space = std::make_shared<ob::RealVectorStateSpace>(3);

        ob::RealVectorBounds bounds(3);
        // x
        bounds.setLow(0, -20.0);
        bounds.setHigh(0,  20.0);
        // y
        bounds.setLow(1, -20.0);
        bounds.setHigh(1,  20.0);
        // z is fixed 0.5
        bounds.setLow(2, 0.5);
        bounds.setHigh(2, 0.5);

        space->setBounds(bounds);

        auto si = std::make_shared<ob::SpaceInformation>(space);

        si->setStateValidityChecker([this](const ob::State *st) {
            return this->isStateValid(st);
        });

        si->setStateValidityCheckingResolution(0.01);

        simple_setup_ = std::make_shared<og::SimpleSetup>(si);
        simple_setup_->setPlanner(std::make_shared<og::RRTstar>(si));
    }

    bool isStateValid(const ob::State *state)
    {
        auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        float x = pos->values[0];
        float y = pos->values[1];
        float z = pos->values[2]; // always 0.5

        pcl::PointXYZ searchPoint(x, y, z);

        float collisionRadius = 0.4f;

        std::vector<int> indices;
        std::vector<float> sqrDistances;

        std::lock_guard<std::mutex> lock(octree_mutex_);
        bool found = octree_.radiusSearch(searchPoint, collisionRadius, indices, sqrDistances);

        return !found; // valid if no obstacle found within collisionRadius
    }

    // --------------------------------------------------------------------------
    //  Cloud Callback: transform to "odom" & fill octree
    // --------------------------------------------------------------------------
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 transformed;
        try
        {
            auto transform_stamped = tf_buffer_->lookupTransform(
                "odom",               // target
                msg->header.frame_id, // source
                tf2::TimePointZero
            );
            tf2::doTransform(*msg, transformed, transform_stamped);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Cloud TF error: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed, *cloud);

        {
            std::lock_guard<std::mutex> lock(octree_mutex_);
            octree_.deleteTree();
            octree_.setInputCloud(cloud);
            octree_.addPointsFromInputCloud();
        }
    }

    // --------------------------------------------------------------------------
    //  Odom Callback: update current_x_, current_y_
    // --------------------------------------------------------------------------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
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
        // If needed: current_z_ = msg->pose.pose.position.z;
    }

    // --------------------------------------------------------------------------
    //  Goal Callback: plan from the robot’s current odom pose => goal
    // --------------------------------------------------------------------------
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double gx = msg->pose.position.x;
        double gy = msg->pose.position.y;

        RCLCPP_INFO(get_logger(),
                    "Received goal (%.2f, %.2f). Planning from robot's odom pose => goal...",
                    gx, gy);

        planPath(gx, gy);
    }

    // --------------------------------------------------------------------------
    //  Plan Path in 3D with z=0.5
    // --------------------------------------------------------------------------
    void planPath(double goal_x, double goal_y)
    {
        // Lock the current odom pose (robot start)
        double start_x, start_y;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            start_x = current_x_;
            start_y = current_y_;
        }

        auto si = simple_setup_->getSpaceInformation();

        // Use the robot’s actual odom pose as the start
        ob::ScopedState<> start(si);
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_y;
        start->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0.5;  // fixed z

        // The goal is (goal_x, goal_y, 0.5)
        ob::ScopedState<> goal(si);
        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_x;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_y;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0.5;

        simple_setup_->clear();
        simple_setup_->setStartAndGoalStates(start, goal);

        ob::PlannerStatus solved = simple_setup_->solve(2.0);

        if (solved && simple_setup_->haveSolutionPath())
        {
            auto path = simple_setup_->getSolutionPath().as<og::PathGeometric>();
            path->interpolate(30);

            // Convert to PoseArray
            geometry_msgs::msg::PoseArray pose_array;
            pose_array.header.stamp = now();
            pose_array.header.frame_id = "odom";

            for (auto &st : path->getStates())
            {
                auto *pos = st->as<ob::RealVectorStateSpace::StateType>();
                geometry_msgs::msg::Pose p;
                p.position.x = pos->values[0];
                p.position.y = pos->values[1];
                p.position.z = pos->values[2];
                p.orientation.w = 1.0; // identity
                pose_array.poses.push_back(p);
            }

            path_pub_->publish(pose_array);

            RCLCPP_INFO(
                get_logger(),
                "Path found (%zu points): start(%.2f,%.2f,0.5) => goal(%.2f,%.2f,0.5).",
                pose_array.poses.size(), start_x, start_y, goal_x, goal_y
            );
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "No path found from (%.2f,%.2f) to (%.2f,%.2f).",
                start_x, start_y, goal_x, goal_y
            );
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planner3D_FixedZ_SubOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
