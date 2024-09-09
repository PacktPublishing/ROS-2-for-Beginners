#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilServerNode : public rclcpp::Node
{
public:
    CountUntilServerNode() : Node("count_until_server")
    {
        count_until_server_ = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServerNode::goalCallback, this, _1, _2),
            std::bind(&CountUntilServerNode::cancelCallback, this, _1),
            std::bind(&CountUntilServerNode::executeCallback, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Action server has been started.");
    }

private:
    // Every new received goal will be processed here first
    // We can decide to accept or reject the incoming goal
    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a goal");
        if (goal->target_number <= 0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting the goal, target number must be positive");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Any cancel request will be processed here, we can accept or reject it
    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received a cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // If a goal has been accepted, it will then be executed in this callback
    // After we are done with the goal execution we set a final state and provide the result
    void executeCallback(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        int target_number = goal_handle->get_goal()->target_number;
        double delay = goal_handle->get_goal()->delay;
        auto result = std::make_shared<CountUntil::Result>();
        int counter = 0;
        rclcpp::Rate loop_rate(1.0/delay);

        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        for (int i = 0; i < target_number; i++) {
            counter++;
            RCLCPP_INFO(this->get_logger(), "%d", counter);
            loop_rate.sleep();
        }

        result->reached_number = counter;
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}