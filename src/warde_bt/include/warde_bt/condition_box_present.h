#ifndef WARDE_BT__CONDITION_BOX_PRESENT_H_
#define WARDE_BT__CONDITION_BOX_PRESENT_H_

#include <behaviortree_cpp_v3/condition_node.h>
#include <memory>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace warde_bt
{

    class ConditionBoxPresent : public BT::ConditionNode
    {
    public:
        ConditionBoxPresent(const std::string &name,
                            const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<std::string>("box_frame"),
                BT::OutputPort<std::string>("front_frame")};
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    };

} // namespace warde_bt

#endif // WARDE_BT__CONDITION_BOX_PRESENT_H_
