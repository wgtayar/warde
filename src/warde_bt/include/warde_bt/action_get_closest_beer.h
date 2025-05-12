#ifndef WARDE_BT__ACTION_GET_CLOSEST_BEER_H_
#define WARDE_BT__ACTION_GET_CLOSEST_BEER_H_

#include <behaviortree_cpp_v3/action_node.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <vector>

namespace warde_bt
{

    class ActionGetClosestBeer : public BT::SyncActionNode
    {
    public:
        ActionGetClosestBeer(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<std::string>>("beer_frames"),
                BT::OutputPort<std::string>("target_beer_frame")};
        }

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;
    };

}

#endif // WARDE_BT__ACTION_GET_CLOSEST_BEER_H_
