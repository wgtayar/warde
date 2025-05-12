#include "warde_bt/condition_box_present.h"
#include <rclcpp/clock.hpp>

namespace warde_bt
{

    ConditionBoxPresent::ConditionBoxPresent(const std::string &name, const BT::NodeConfiguration &config) : BT::ConditionNode(name, config)
    {
        auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }

    BT::NodeStatus ConditionBoxPresent::tick()
    {
        try
        {
            buffer_->lookupTransform("odom", "box_frame", rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.1));

            setOutput("box_frame", std::string("box_frame"));
            RCLCPP_INFO(rclcpp::get_logger("ConditionBoxPresent"), "Found box frame");
            return BT::NodeStatus::SUCCESS;
        }
        catch (const tf2::TransformException &)
        {
            RCLCPP_WARN(rclcpp::get_logger("ConditionBoxPresent"), "No box frame found");
            return BT::NodeStatus::FAILURE;
        }
    }

}
