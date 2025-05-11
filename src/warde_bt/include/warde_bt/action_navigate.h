#ifndef WARDE_BT__ACTION_NAVIGATE_H_
#define WARDE_BT__ACTION_NAVIGATE_H_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_nav/action/navigate.hpp"

namespace warde_bt
{

    using Navigate = robot_nav::action::Navigate;
    using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;

    class ActionNavigate : public BT::StatefulActionNode
    {
    public:
        ActionNavigate(const std::string &name, const BT::NodeConfiguration &cfg);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("wander", "true=roam; false=go to target_frame"),
                BT::InputPort<std::string>("target_frame", "TF frame to navigate to")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<Navigate>::SharedPtr client_;
        std::shared_future<GoalHandle::SharedPtr> gh_future_;
        GoalHandle::SharedPtr goal_handle_;
        bool active_{false};
    };

} // namespace warde_bt
#endif // WARDE_BT__ACTION_NAVIGATE_H_
