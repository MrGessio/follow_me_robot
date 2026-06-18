#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>

class FollowMeNode : public rclcpp::Node
{
public:
    FollowMeNode() : Node("follow_me_node")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FollowMeNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "FollowMeNode has been started");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto ranges = msg->ranges;
        int range_size = ranges.size();

        // Find the closest valid reading and its index
        float min_distance = std::numeric_limits<float>::infinity();
        int min_index = -1;

        for (int i = 0; i < range_size; i++) {
            float r = ranges[i];
            if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
                if (r < min_distance) {
                    min_distance = r;
                    min_index = i;
                }
            }
        }

        geometry_msgs::msg::Twist cmd;

        if (min_index == -1) {
            // No valid readings — stop
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            return;
        }

        // Convert index to angle (in radians)
        float angle = msg->angle_min + min_index * msg->angle_increment;

        // Normalize angle to [-pi, pi]
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;

        // Control parameters
        const float target_distance = 0.6;   // meters — desired following distance
        const float stop_distance   = 0.3;   // meters — too close, stop
        const float linear_speed    = 0.2;   // m/s
        const float angular_speed   = 0.5;   // rad/s
        const float angle_threshold = 0.15;  // rad — dead zone for rotation (~8 degrees)

        if (min_distance < stop_distance) {
            // Object too close — stop
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        } else {
            // Rotate toward object
            if (std::abs(angle) > angle_threshold) {
                cmd.angular.z = (angle > 0) ? angular_speed : -angular_speed;
            } else {
                cmd.angular.z = 0.0;
            }

            // Move forward if object is farther than target distance
            if (min_distance > target_distance) {
                cmd.linear.x = linear_speed;
            } else {
                cmd.linear.x = 0.0;
            }
        }

        cmd_vel_pub_->publish(cmd);

        RCLCPP_DEBUG(this->get_logger(),
            "Closest object: %.2f m at angle %.2f rad | linear: %.2f | angular: %.2f",
            min_distance, angle, cmd.linear.x, cmd.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowMeNode>());
    rclcpp::shutdown();
    return 0;
}
