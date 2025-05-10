#include "warde_bt/action_manipulate.h"

using namespace std::chrono_literals;

namespace warde_bt
{

    ActionManipulate::ActionManipulate(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_manipulate_client");
        client_ = node_->create_client<manipulation_msgs::srv::Manipulate>("manipulation_service");
    }

    BT::NodeStatus ActionManipulate::tick()
    {
        std::string manipulation, pre_frame, target_frame;
        if (!getInput("manipulation", manipulation))
        {
            throw BT::RuntimeError("ActionManipulate missing [manipulation] port");
        }
        if (!getInput("pre_target_frame", pre_frame))
        {
            throw BT::RuntimeError("ActionManipulate missing [pre_target_frame] port");
        }
        if (!getInput("target_frame", target_frame))
        {
            throw BT::RuntimeError("ActionManipulate missing [target_frame] port");
        }

        if (!client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(node_->get_logger(), "manipulation_service not available");
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<manipulation_msgs::srv::Manipulate::Request>();
        req->manipulation = manipulation;
        req->pre_target_frame = pre_frame;
        req->target_frame = target_frame;

        auto future = client_->async_send_request(req);

        if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "manipulation_service call failed");
            return BT::NodeStatus::FAILURE;
        }

        auto res = future.get();
        if (!res->success)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "manipulation failed: %s",
                res->message.c_str());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "manipulation succeeded: %s",
            res->message.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // BT_REGISTER_NODES(factory)
    // {
    //     factory.registerNodeType<ActionManipulate>("ActionManipulate");
    // }

}