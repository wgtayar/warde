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
        node_ = rclcpp::Node::make_shared("bt_navigate_client");
        navigate_client_ = node_->create_client<robot_nav::srv::Navigate>("navigate");
    }

    BT::NodeStatus ActionNavigate::onStart()
    {
        if (!navigate_client_->wait_for_service(1s))
        {
            RCLCPP_ERROR(node_->get_logger(), "navigate service unavailable");
            return BT::NodeStatus::FAILURE;
        }

        bool wander = false;
        getInput("wander", wander);

        std::string target;
        getInput("target_frame", target);

        auto req = std::make_shared<robot_nav::srv::Navigate::Request>();
        req->wander = wander;
        req->target_frame = target;

        navigate_client_->async_send_request(req);
        active_ = true;
        return BT::NodeStatus::RUNNING;
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

    // BT_REGISTER_NODES(factory)
    // {
    //     factory.registerNodeType<ActionNavigate>("ActionNavigate");
    // }

}