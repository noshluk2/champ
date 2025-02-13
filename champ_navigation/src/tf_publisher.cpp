#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TFBroadcaster : public rclcpp::Node
{
public:
    TFBroadcaster() : Node("tf_broadcaster")
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TFBroadcaster::broadcastTransform, this)
        );
    }
    // ros2 run tf2_ros static_transform_publisher 0 0 0 -0.9 0.0 -1.57 odom camera_depth_optical_frame

private:
    void broadcastTransform()
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        // Fill the transformation data
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "camera_depth_optical_frame";

        // Translation
        transformStamped.transform.translation.x = 0.175;
        transformStamped.transform.translation.y = 0.105;
        transformStamped.transform.translation.z = 0.0;

        // Rotation (Quaternion)
        transformStamped.transform.rotation.x = -3.6756855337216614e-08;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 0.9999999999999993;

        // Publish the transform
        broadcaster_->sendTransform(transformStamped);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
