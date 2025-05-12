#include "warde_bt/action_navigate.h"
#include <chrono>

using namespace std::chrono_literals;

namespace warde_bt
{
    ActionNavigate::ActionNavigate(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
    }

    BT::NodeStatus ActionNavigate::onStart()
    {
        getInput("ros_node", node_);

        if (!navigate_client_)
        {
            navigate_client_ = node_->create_client<robot_nav::srv::Navigate>(
                "navigate", rmw_qos_profile_services_default);
        }

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

        auto result_future_ = navigate_client_->async_send_request(req);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ActionNavigate::onRunning()
    {
        if (!result_future_.valid())
        {
            return BT::NodeStatus::FAILURE;
        }

        if (result_future_.wait_for(0s) == std::future_status::ready)
        {
            auto res = result_future_.get();
            return (res->success ? BT::NodeStatus::SUCCESS
                                 : BT::NodeStatus::FAILURE);
        }

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
