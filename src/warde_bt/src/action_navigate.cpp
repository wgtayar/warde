#include "warde_bt/action_navigate.h"
#include <chrono>
using namespace std::chrono_literals;

namespace warde_bt
{

    ActionNavigate::ActionNavigate(
        const std::string &name,
        const BT::NodeConfiguration &cfg)
        : BT::StatefulActionNode(name, cfg)
    {
        node_ = rclcpp::Node::make_shared("bt_navigate_client");
        client_ = rclcpp_action::create_client<Navigate>(node_, "navigate");
    }

    BT::NodeStatus ActionNavigate::onStart()
    {
        if (!client_->wait_for_action_server(1s))
        {
            RCLCPP_ERROR(node_->get_logger(), "Navigate action not available");
            return BT::NodeStatus::FAILURE;
        }
        bool wander;
        getInput("wander", wander);
        std::string target;
        getInput("target_frame", target);

        Navigate::Goal goal_msg;
        goal_msg.wander = wander;
        goal_msg.target_frame = target;

        gh_future_ = client_->async_send_goal(goal_msg);
        active_ = true;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ActionNavigate::onRunning()
    {
        if (!active_)
            return BT::NodeStatus::FAILURE;

        if (!goal_handle_ &&
            gh_future_.valid() &&
            gh_future_.wait_for(0s) == std::future_status::ready)
        {
            goal_handle_ = gh_future_.get();
            if (!goal_handle_)
            {
                RCLCPP_ERROR(node_->get_logger(), "Navigate goal was rejected");
                return BT::NodeStatus::FAILURE;
            }
        }

        if (goal_handle_)
        {
            auto result_future = client_->async_get_result(goal_handle_);
            if (result_future.wait_for(0s) == std::future_status::ready)
            {
                auto res = result_future.get().result;
                return res->success ? BT::NodeStatus::SUCCESS
                                    : BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void ActionNavigate::onHalted()
    {
        if (active_ && goal_handle_)
        {
            client_->async_cancel_goal(goal_handle_);
        }
        active_ = false;
    }

} // namespace warde_bt