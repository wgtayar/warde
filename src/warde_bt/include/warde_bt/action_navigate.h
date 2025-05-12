#ifndef WARDE_BT__ACTION_NAVIGATE_H_
#define WARDE_BT__ACTION_NAVIGATE_H_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_nav/action/navigate.hpp>

namespace warde_bt
{

    class ActionNavigate : public BT::StatefulActionNode
    {
    public:
        using Navigate = robot_nav::action::Navigate;
        using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;
        using WrappedResult =
            typename rclcpp_action::Client<Navigate>::WrappedResult;

        ActionNavigate(const std::string &name,
                       const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("wander"),
                BT::InputPort<std::string>("target_frame")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<Navigate>::SharedPtr client_;

        std::shared_future<typename GoalHandle::SharedPtr> goal_handle_future_;
        typename GoalHandle::SharedPtr goal_handle_;

        std::shared_future<WrappedResult> result_future_;

        bool wander_{true};
        std::string target_;
        bool active_{false};
    };

} // namespace warde_bt

#endif // WARDE_BT__ACTION_NAVIGATE_H_