#ifndef WARDE_BT__ACTION_MANIPULATE_H_
#define WARDE_BT__ACTION_MANIPULATE_H_

#include <string>
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "manipulation_msgs/srv/manipulate.h"

namespace warde_bt
{

    class ActionManipulate : public BT::SyncActionNode
    {
    public:
        ActionManipulate(const std::string &name, const BT::NodeConfiguration &config);

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("manipulation", "pick or place"),
                BT::InputPort<std::string>("pre_target_frame", "the frame to go to before the final pose"),
                BT::InputPort<std::string>("target_frame", "the final grasp/place frame")};
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<manipulation_msgs::srv::Manipulate>::SharedPtr client_;
    };

}

#endif // WARDE_BT__ACTION_MANIPULATE_H_