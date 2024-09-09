#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilClientNode : public rclcpp::Node
{
public:
    CountUntilClientNode() : Node("count_until_client")
    {
        count_until_client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
    }

    // Wait for action server, send a goal, and register callbacks for the response and the result
    // Also register another callback for the optional feedback
    void sendGoal(int target_number, double delay)
    {
        count_until_client_->wait_for_action_server();
        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.delay = delay;
        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.goal_response_callback = std::bind(&CountUntilClientNode::goalResponseCallback, this, _1);
        options.result_callback = std::bind(&CountUntilClientNode::goalResultCallback, this, _1);
        options.feedback_callback = std::bind(&CountUntilClientNode::goalFeedbackCallback, this, _1, _2);
        count_until_client_->async_send_goal(goal, options);
    }

    // Method to send a cancel request
    void cancelGoal()
    {
        RCLCPP_INFO(this->get_logger(), "Send a cancel goal request");
        count_until_client_->async_cancel_all_goals();
    }

private:
    // Get the goal response and print it
    void goalResponseCallback(const CountUntilGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }
    
    // Get the goal result and print it
    void goalResultCallback(const CountUntilGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED) {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }
        int reached_number = result.result->reached_number;
        RCLCPP_INFO(this->get_logger(), "Result: %d", reached_number);
    }

    // Get the goal feedback and print it
    void goalFeedbackCallback(const CountUntilGoalHandle::SharedPtr &goal_handle,
        const std::shared_ptr<const CountUntil::Feedback> feedback)
    {
        (void)goal_handle;
        int number = feedback->current_number;
        RCLCPP_INFO(this->get_logger(), "Got feedback: %d", number);
        // if (number >= 2) {
        //     cancelGoal();
        // }
    }

    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>();
    node->sendGoal(5, 0.5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}