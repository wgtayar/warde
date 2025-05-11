#include "warde_bt/action_navigate.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <chrono>

using namespace std::chrono_literals;

namespace warde_bt
{
    ActionNavigate::ActionNavigate(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_navigate_client");
        navigate_client_ = node_->create_client<robot_nav::srv::Navigate>("navigate", rmw_qos_profile_services_default);
    }

    BT::NodeStatus ActionNavigate::onStart()
    {
        if (!navigate_client_->wait_for_service(1s))
        {
            RCLCPP_ERROR(node_->get_logger(), "navigate service unavailable");
            return BT::NodeStatus::FAILURE;
        }

        bool wander = true;
        getInput("wander", wander);

        std::string target;
        getInput("target_frame", target);

        auto req = std::make_shared<robot_nav::srv::Navigate::Request>();
        req->wander = wander;
        req->target_frame = target;

        auto result_future = navigate_client_->async_send_request(req);
        auto status = rclcpp::spin_until_future_complete(node_, result_future);

        if (status == rclcpp::FutureReturnCode::SUCCESS && result_future.get()->success)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "navigate call failed or timed out");
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus ActionNavigate::onRunning()
    {
        return BT::NodeStatus::RUNNING;
    }

    void ActionNavigate::onHalted()
    {
        if (active_)
        {
            auto req = std::make_shared<robot_nav::srv::Navigate::Request>();
            req->wander = false;
            req->target_frame = "";
            navigate_client_->async_send_request(req);
            active_ = false;
        }
    }
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<warde_bt::ActionNavigate>("ActionNavigate");
// }