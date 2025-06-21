#include "rclcpp/rclcpp.hpp"

class FollowMeNode : public rclcpp::Node
{
public:
    FollowMeNode() : Node("follow_me_node")
    {
        RCLCPP_INFO(this->get_logger(), "FollowMeNode has been started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowMeNode>());
    rclcpp::shutdown();
    return 0;
}

