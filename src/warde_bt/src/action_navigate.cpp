#include "warde_bt/action_navigate.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <chrono>

using namespace std::chrono_literals;

namespace warde_bt
{
    BT::PortsList ActionNavigate::providedPorts()
    {
        return {
            BT::InputPort<bool>("wander", true, "whether to wander or not; default true"),
            BT::InputPort<std::string>("target_frame", "", "target frame for navigation; default empty")};
    }

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
        bool wander_flag = true;
        getInput("wander", wander_flag);

        std::string target;
        getInput("target_frame", target);

        if (!navigate_client_->wait_for_service(1s))
        {
            RCLCPP_ERROR(node_->get_logger(), "navigate service unavailable");
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<robot_nav::srv::Navigate::Request>();
        req->wander = wander_flag;
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
}

// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<warde_bt::ActionNavigate>("ActionNavigate");
// }