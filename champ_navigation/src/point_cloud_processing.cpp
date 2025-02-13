#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("pointcloud_processor") {
        depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10,
            std::bind(&PointCloudProcessor::depthCallback, this, std::placeholders::_1));

        filtered_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "filtered_point_cloud", 10);
            RCLCPP_INFO(this->get_logger(), "Point Cloud processing node started");
    }

private:
    void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud->size());

        // Apply voxel grid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(0.2f, 0.2f, 0.2f);
        voxel.filter(*voxel_filtered_cloud);

        // Publish voxel-filtered point cloud
        sensor_msgs::msg::PointCloud2 voxel_msg;
        pcl::toROSMsg(*voxel_filtered_cloud, voxel_msg);
        voxel_msg.header.stamp = msg->header.stamp;
        voxel_msg.header.frame_id = "camera_depth_optical_frame";
        filtered_point_cloud_pub_->publish(voxel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_point_cloud_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
