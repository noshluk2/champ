#include <chrono>
#include <mutex>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>


#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class DynamicPlannerNode : public rclcpp::Node
{
public:
    DynamicPlannerNode()
      : Node("dynamic_planner"),
        octree_(0.1f)
    {
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/planner/path", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&DynamicPlannerNode::odomCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/front/points", 10,
            std::bind(&DynamicPlannerNode::depthCallback, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        initializePlanner();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_;
    std::mutex octree_mutex_;

    geometry_msgs::msg::Pose current_pose_;
    std::vector<geometry_msgs::msg::Pose> current_path_;
    std::mutex pose_mutex_;

    og::SimpleSetupPtr simple_setup_;

    double goal_x_ = 3.0; // Fixed goal X
    double goal_y_ = 3.0; // Fixed goal Y

    void initializePlanner()
    {
        auto space = std::make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, -20.0);
        bounds.setHigh(0, 20.0);
        bounds.setLow(1, -20.0);
        bounds.setHigh(1, 20.0);
        space->setBounds(bounds);

        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker([this](const ob::State *state) {
            return this->isStateValid(state);
        });
        si->setMotionValidator(std::make_shared<ob::DiscreteMotionValidator>(si));
        si->setStateValidityCheckingResolution(0.005);

        simple_setup_ = std::make_shared<og::SimpleSetup>(si);
        simple_setup_->setPlanner(std::make_shared<og::RRTstar>(si));
    }

    bool isStateValid(const ob::State *state)
    {
        auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        pcl::PointXYZ point(pos->values[0], pos->values[1], 1.0);
        std::vector<int> idx_search;
        std::vector<float> dist_search;
        std::lock_guard<std::mutex> lock(octree_mutex_);
        return !octree_.radiusSearch(point, 1.0, idx_search, dist_search);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = msg->pose.pose;
        if (current_path_.empty()) computeNewPath();
    }

    void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        std::lock_guard<std::mutex> lock(octree_mutex_);
        octree_.deleteTree();
        octree_.setInputCloud(cloud);
        octree_.addPointsFromInputCloud();

        if (!pathIsValid()) computeNewPath();
    }

    bool pathIsValid()
    {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        for (auto &pose : current_path_)
        {
            pcl::PointXYZ point(pose.position.x, pose.position.y, 1.0);
            std::vector<int> idx_search;
            std::vector<float> dist_search;
            if (octree_.radiusSearch(point, 1.0, idx_search, dist_search)) return false;
        }
        return true;
    }

    void computeNewPath()
    {
        ob::ScopedState<> start(simple_setup_->getSpaceInformation());
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = current_pose_.position.x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = current_pose_.position.y;

        ob::ScopedState<> goal(simple_setup_->getSpaceInformation());
        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_x_;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_y_;

        simple_setup_->clear();
        simple_setup_->setStartAndGoalStates(start, goal);
        simple_setup_->solve(1.0);

        if (simple_setup_->haveSolutionPath())
        {
            current_path_.clear();
            auto path = simple_setup_->getSolutionPath().as<og::PathGeometric>();
            path->interpolate();

            geometry_msgs::msg::PoseArray pose_array;
            pose_array.header.stamp = this->now();
            pose_array.header.frame_id = "base_link";

            for (auto &state : path->getStates())
            {
                auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
                geometry_msgs::msg::Pose p;
                p.position.x = pos->values[0];
                p.position.y = pos->values[1];
                current_path_.emplace_back(p);
                pose_array.poses.emplace_back(p);
            }
            path_pub_->publish(pose_array);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}