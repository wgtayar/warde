// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// THIS IS A DUMMY IMPLEMENTATION OF THE ACTION MANIPULATE CLASS, WAITING FOR THE ACTUAL IMPLEMENTATION TO BE READY.
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#ifndef WARDE_BT__ACTION_MANIPULATE_H_
#define WARDE_BT__ACTION_MANIPULATE_H_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace warde_bt
{

    class ActionManipulate : public BT::SyncActionNode
    {
    public:
        ActionManipulate(const std::string &name, const BT::NodeConfiguration &config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("manipulation", "pick/place"),
                BT::InputPort<std::string>("pre_target_frame", "pre-grasp/place frame"),
                BT::InputPort<std::string>("target_frame", "actual grasp/place frame")};
        }

        BT::NodeStatus tick() override
        {
            std::string manipulation, pre, target;
            getInput("manipulation", manipulation);
            getInput("pre_target_frame", pre);
            getInput("target_frame", target);

            RCLCPP_INFO(
                rclcpp::get_logger("ActionManipulate"),
                "[%s] manipulation=%s, pre=\'%s\', target=\'%s\' → SUCCESS",
                name().c_str(),
                manipulation.c_str(), pre.c_str(), target.c_str());

            return BT::NodeStatus::SUCCESS;
        }
    };

} // namespace warde_bt

#endif // WARDE_BT__ACTION_MANIPULATE_H_

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ANOTHER DUMMY IMPLEMENTATION, MAY BE MODIFIED LATER
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// #ifndef WARDE_BT__ACTION_MANIPULATE_H_
// #define WARDE_BT__ACTION_MANIPULATE_H_

// #include <string>
// #include <behaviortree_cpp_v3/action_node.h>
// #include <rclcpp/rclcpp.hpp>
// #include "manipulation_msgs/srv/manipulate.h"

// namespace warde_bt
// {

//     class ActionManipulate : public BT::SyncActionNode
//     {
//     public:
//         ActionManipulate(const std::string &name, const BT::NodeConfiguration &config);

//         BT::NodeStatus tick() override;

//         static BT::PortsList providedPorts()
//         {
//             return {
//                 BT::InputPort<std::string>("manipulation", "pick or place"),
//                 BT::InputPort<std::string>("pre_target_frame", "the frame to go to before the final pose"),
//                 BT::InputPort<std::string>("target_frame", "the final grasp/place frame")};
//         }

//     private:
//         rclcpp::Node::SharedPtr node_;
//         rclcpp::Client<manipulation_msgs::srv::Manipulate>::SharedPtr client_;
//     };

// }

// #endif // WARDE_BT__ACTION_MANIPULATE_H_

