#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class FollowMeNode : public rclcpp::Node
{
public:
    FollowMeNode() : Node("follow_me_node")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        image_sub_ = image_transport::create_subscription(
            this, "/camera/image_raw",
            std::bind(&FollowMeNode::imageCallback, this, std::placeholders::_1),
            "raw");

        RCLCPP_INFO(this->get_logger(), "FollowMeNode started - tracking red objects");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        int frame_width = frame.cols;
        int frame_height = frame.rows;

        // Convert to HSV
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Threshold for red color (two ranges in HSV)
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, 120, 70),   cv::Scalar(10, 255, 255),  mask1);
        cv::inRange(hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255), mask2);
        cv::Mat mask = mask1 | mask2;

        // Morphological cleanup
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        geometry_msgs::msg::Twist cmd;

        if (!contours.empty())
        {
            // Find largest contour (assumed to be the target object)
            auto largest_contour = *std::max_element(
                contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                    return cv::contourArea(c1) < cv::contourArea(c2);
                });

            double area = cv::contourArea(largest_contour);

            // Ignore very small detections (noise)
            if (area > 500.0)
            {
                cv::Rect bounding_box = cv::boundingRect(largest_contour);
                cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 2);

                // Calculate center of detected object
                int obj_center_x = bounding_box.x + bounding_box.width / 2;

                // Normalized error: -1.0 (far left) to +1.0 (far right)
                double error = (double)(obj_center_x - frame_width / 2) / (frame_width / 2);

                // Angular control - rotate toward object
                const double angular_gain = 0.8;
                cmd.angular.z = -angular_gain * error;

                // Linear control - move forward if object is small (far away)
                // Stop if object fills too much of the frame (too close)
                double area_ratio = area / (frame_width * frame_height);
                const double target_area_ratio = 0.05;  // ~5% of frame = desired distance

                if (area_ratio < target_area_ratio) {
                    cmd.linear.x = 0.15;  // move forward
                } else {
                    cmd.linear.x = 0.0;   // close enough, stop
                }

                RCLCPP_DEBUG(this->get_logger(),
                    "Target detected | area: %.0f | error: %.2f | linear: %.2f | angular: %.2f",
                    area, error, cmd.linear.x, cmd.angular.z);
            }
            else
            {
                // Object too small / lost - stop
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }
        }
        else
        {
            // No object detected - stop
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        cmd_vel_pub_->publish(cmd);
        cv::imshow("Follow Me Camera", frame);
        cv::waitKey(1);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    image_transport::Subscriber image_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowMeNode>());
    rclcpp::shutdown();
    return 0;
}
