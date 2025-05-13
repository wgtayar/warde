#include "warde_bt/action_navigate.h"
#include "warde_bt/condition_beer_present.h"

using namespace std::chrono_literals;

namespace warde_bt
{

    ActionNavigate::ActionNavigate(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = rclcpp_action::create_client<Navigate>(node_, "navigate");
    }

    BT::NodeStatus ActionNavigate::onStart()
    {
        RCLCPP_INFO(node_->get_logger(), ">> ActionNavigate::onStart()");

        goal_handle_.reset();
        result_future_.~shared_future();

        if (!client_->wait_for_action_server(1s))
        {
            RCLCPP_ERROR(node_->get_logger(), " Navigate action server unavailable");
            return BT::NodeStatus::FAILURE;
        }

        getInput("wander", wander_);
        getInput("target_frame", target_);
        RCLCPP_INFO(node_->get_logger(), " params: wander=%s target_frame='%s'", wander_ ? "true" : "false", target_.c_str());

        Navigate::Goal goal_msg;
        goal_msg.wander = wander_;
        goal_msg.target_frame = target_;

        RCLCPP_INFO(node_->get_logger(), " sending goal");
        rclcpp_action::Client<Navigate>::SendGoalOptions options;
        goal_handle_future_ = client_->async_send_goal(goal_msg, options);

        active_ = true;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ActionNavigate::onRunning()
    {
        RCLCPP_INFO(node_->get_logger(), ">> ActionNavigate::onRunning()");

        if (!active_)
            return BT::NodeStatus::FAILURE;

        if (!goal_handle_)
        {
            if (goal_handle_future_.wait_for(0s) == std::future_status::ready)
            {
                goal_handle_ = goal_handle_future_.get();
                if (!goal_handle_)
                {
                    RCLCPP_ERROR(node_->get_logger(), " goal was rejected");
                    return BT::NodeStatus::FAILURE;
                }

                RCLCPP_INFO(node_->get_logger(),
                            "   got goal handle %p",
                            static_cast<void *>(goal_handle_.get()));

                result_future_ = client_->async_get_result(goal_handle_);
                RCLCPP_INFO(node_->get_logger(), " async_get_result() called");
            }
            else
            {
                return BT::NodeStatus::RUNNING;
            }
        }

        if (result_future_.wait_for(0s) == std::future_status::ready)
        {
            auto wrapped = result_future_.get();
            active_ = false;

            if (!wander_ && wrapped.result->success && target_.rfind("beer", 0) == 0)
            {
                ConditionBeerPresent::picked_beers_.push_back(target_);
                RCLCPP_INFO(node_->get_logger(),
                            "Tagged beer '%s' as picked", target_.c_str());
            }

            return wrapped.result->success
                       ? BT::NodeStatus::SUCCESS
                       : BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void ActionNavigate::onHalted()
    {
        RCLCPP_INFO(node_->get_logger(), ">> ActionNavigate::onHalted()");
        if (active_ && goal_handle_)
        {
            RCLCPP_INFO(node_->get_logger(), " cancelling goal");
            client_->async_cancel_goal(goal_handle_);
            active_ = false;
            goal_handle_.reset();
            result_future_.~shared_future();
        }
    }

} // namespace warde_bt