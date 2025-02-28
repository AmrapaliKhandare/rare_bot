#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



class ESPProcessor : public rclcpp::Node {
public:
    ESPProcessor() : Node("esp_processor") {
        // subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //     "esp_data", 10, std::bind(&ESPProcessor::data_callback, this, std::placeholders::_1));

        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/micro_ros_wifi_publisher", 10, std::bind(&ESPProcessor::data_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("bot_pose", 10);
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", 10);
        
        cloud_accumulated_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();


        // Publish the map frame at the origin
        // publish_map_origin();

    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accumulated_;


    void publish_map_origin() {
        geometry_msgs::msg::TransformStamped map_transform;
    
        map_transform.header.stamp = this->now();
        map_transform.header.frame_id = "map";  // or "map", depending on your context
        map_transform.child_frame_id = "base_link";    // This will be the map frame at the origin
    
        map_transform.transform.translation.x = 0.5;
        map_transform.transform.translation.y = 0.5;
        map_transform.transform.translation.z = 0.0;
        map_transform.transform.rotation.w = 1.0;  // No rotation, identity rotation
    
        tf_broadcaster_->sendTransform(map_transform);
    }
    
    void data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Incomplete data received!");
            return;
        }

        float bot_x = msg->data[0];
        float bot_y = msg->data[1];
        float bot_ang = msg->data[2];
        float ping_ang = msg->data[3];
        float ping_dist = msg->data[4]/1000;

        // Publish Pose
        // auto pose_msg = geometry_msgs::msg::PoseStamped();
        // geometry_msgs::msg::TransformStamped pose_msg;
        // pose_msg.header.stamp = this->now();
        // pose_msg.header.frame_id = "map";
        // pose_msg.child_frame_id = "base_link";    // This will be the map frame at the origin
        // pose_msg.pose.position.x = bot_x;
        // pose_msg.pose.position.y = bot_y;
        // pose_msg.pose.orientation.z = sin(bot_ang / 2.0);
        // pose_msg.pose.orientation.w = cos(bot_ang / 2.0);
        // pose_pub_->publish(pose_msg);

        geometry_msgs::msg::TransformStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.child_frame_id = "base_link";
        pose_msg.transform.translation.x = bot_x/1000.0;
        pose_msg.transform.translation.y = bot_y/1000.0;

        double roll_bot= 0.0, pitch_bot = 0.0, yaw_bot = bot_ang * M_PI /180.0;
        tf2::Quaternion q_bot;
        RCLCPP_INFO(this->get_logger(), "yaw_bot: %.4f rad", yaw_bot);
        q_bot.setRPY(roll_bot, pitch_bot, yaw_bot);  // Set roll (X), pitch (Y), yaw (Z)
        // geometry_msgs::msg::Quaternion quat_msg_bot;
        // pose_msg.transform.translation.z = 0.05;
        pose_msg.transform.rotation.x = q_bot.x();
        pose_msg.transform.rotation.y = q_bot.y();
        pose_msg.transform.rotation.z = q_bot.z();
        pose_msg.transform.rotation.w = q_bot.w();

        RCLCPP_INFO(this->get_logger(), "bot quat x: %.4f, y: %.4f, z: %.4f, w: %.4f", pose_msg.transform.rotation.x, pose_msg.transform.rotation.y, pose_msg.transform.rotation.z, pose_msg.transform.rotation.w);

        tf_broadcaster_->sendTransform(pose_msg);

        // Publish TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "laser_frame";
        tf_msg.transform.translation.x = 0.1;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.05;
        // tf_msg.transform.rotation.w = 1.0;

        double roll_ping= 0.0, pitch_ping = 0.0, yaw_ping= ping_ang * M_PI /180.0;
        tf2::Quaternion q_ping;
        RCLCPP_INFO(this->get_logger(), "yaw_ping: %.4f rad", yaw_ping);
        q_ping.setRPY(roll_ping, pitch_ping, yaw_ping);  // Set roll (X), pitch (Y), yaw (Z)
        // geometry_msgs::msg::Quaternion quat_msg_ping;
        // pose_msg.transform.translation.z = 0.05;
        tf_msg.transform.rotation.x = q_ping.x();
        tf_msg.transform.rotation.y = q_ping.y();
        tf_msg.transform.rotation.z = q_ping.z();
        tf_msg.transform.rotation.w = q_ping.w();

        RCLCPP_INFO(this->get_logger(), "bot quat x: %.4f, y: %.4f, z: %.4f, w: %.4f", tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w);

        tf_broadcaster_->sendTransform(tf_msg);

        // Publish LaserScan
        // auto laser_msg = sensor_msgs::msg::LaserScan();
        // laser_msg.header.stamp = this->now();
        // laser_msg.header.frame_id = "laser_frame";
        // laser_msg.angle_min = ping_ang;
        // laser_msg.angle_max = ping_ang;
        // laser_msg.angle_increment = 0.0;
        // laser_msg.range_min = 0.02;
        // laser_msg.range_max = 3.0;
        // laser_msg.ranges = {ping_dist};
        // laser_pub_->publish(laser_msg);

        if (ping_dist <= 0.02 || ping_dist > 2.0) return;  // Filter out invalid data
        
        // Convert angle to radians
        float rad_ang = ping_ang * M_PI / 180.0;

        // Create PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointXYZ point;

        point.x = ping_dist * cos(rad_ang);
        point.y = ping_dist * sin(rad_ang);
        point.z = 0.05;  // Height of ultrasonic sensor

        // cloud.points.push_back(point);

        // // Convert to ROS msg
        // sensor_msgs::msg::PointCloud2 cloud_msg;
        // pcl::toROSMsg(cloud, cloud_msg);
        // cloud_msg.header.stamp = this->now();
        // cloud_msg.header.frame_id = "laser_frame";  // Same frame as TF

        // pcl_pub_->publish(cloud_msg);
        // RCLCPP_INFO(this->get_logger(), "Published PointCloud: X=%.3f Y=%.3f", point.x, point.y);
        // Append point to cloud
        cloud_accumulated_->points.push_back(point);

        // Publish accumulated cloud
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_accumulated_, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "laser_frame";

        pcl_pub_->publish(cloud_msg);

        RCLCPP_INFO(this->get_logger(), "Total Points: %lu", cloud_accumulated_->points.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ESPProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

