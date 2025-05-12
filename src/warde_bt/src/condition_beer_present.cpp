#include "warde_bt/condition_beer_present.h"
#include <rclcpp/clock.hpp>

namespace warde_bt
{

    ConditionBeerPresent::ConditionBeerPresent(const std::string &name, const BT::NodeConfiguration &config) : BT::ConditionNode(name, config)
    {
        auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }

    BT::NodeStatus ConditionBeerPresent::tick()
    {
        auto frames = buffer_->getAllFrameNames();
        std::vector<std::string> beers;
        for (auto &f : frames)
        {
            if (f.rfind("beer", 0) == 0)
                beers.push_back(f);
        }

        if (beers.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("ConditionBeerPresent"), "No beer frames found");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("beer_frames", beers);
        RCLCPP_INFO(rclcpp::get_logger("ConditionBeerPresent"), "Found %zu beer frames", beers.size());
        return BT::NodeStatus::SUCCESS;
    }

} // namespace warde_bt