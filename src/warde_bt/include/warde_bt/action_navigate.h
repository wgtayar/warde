#ifndef WARDE_BT__ACTION_NAVIGATE_H_
#define WARDE_BT__ACTION_NAVIGATE_H_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_nav/srv/navigate.hpp>

namespace warde_bt
{

    class ActionNavigate : public BT::StatefulActionNode
    {
    public:
        ActionNavigate(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<rclcpp::Node::SharedPtr>("ros_node"),
                BT::InputPort<bool>("wander", "true=aimless wander; false=go to target_frame"),
                BT::InputPort<std::string>("target_frame", "TF frame to navigate to")};
        }

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<robot_nav::srv::Navigate>::SharedPtr navigate_client_;
        rclcpp::Client<robot_nav::srv::Navigate>::SharedFuture result_future_;
        bool active_{false};
    };

}

#endif // WARDE_BT__ACTION_NAVIGATE_H_