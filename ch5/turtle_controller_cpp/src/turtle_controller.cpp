#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::placeholders;

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleControllerNode::poseCallback, this, _1));
    }

    void poseCallback(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto cmd = geometry_msgs::msg::Twist();
        if (pose->x < 5.5) {
            cmd.linear.x = 1.0;
            cmd.angular.z = 1.0;
        }
        else {
            cmd.linear.x = 2.0;
            cmd.angular.z = 2.0;
        }
        cmd_vel_pub_->publish(cmd);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}