#ifndef WARDE_BT__CONDITION_BEER_PRESENT_H_
#define WARDE_BT__CONDITION_BEER_PRESENT_H_

#include <behaviortree_cpp_v3/condition_node.h>
#include <memory>
#include <string>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace warde_bt
{

    class ConditionBeerPresent : public BT::ConditionNode
    {
    public:
        ConditionBeerPresent(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<std::vector<std::string>>("beer_frames")};
        }

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;
    };

} // namespace warde_bt

#endif // WARDE_BT__CONDITION_BEER_PRESENT_H_
