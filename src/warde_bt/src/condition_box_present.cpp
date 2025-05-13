#include "warde_bt/condition_box_present.h"
#include <rclcpp/clock.hpp>

namespace warde_bt
{

    ConditionBoxPresent::ConditionBoxPresent(const std::string &name,
                                             const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

        auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    }

    BT::NodeStatus ConditionBoxPresent::tick()
    {
        try
        {
            buffer_->lookupTransform("odom",
                                     "box_frame",
                                     rclcpp::Time(0, 0, RCL_ROS_TIME),
                                     rclcpp::Duration::from_seconds(0.1));

            setOutput("box_frame", std::string("box_frame"));
            setOutput("front_frame", std::string("box_front_frame"));
            RCLCPP_INFO(node_->get_logger(), "Found box_frame; broadcasting box_front_frame");

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = node_->now();
            tf.header.frame_id = "box_frame";
            tf.child_frame_id = "box_front_frame";
            tf.transform.translation.x = 0.0;
            tf.transform.translation.y = 0.4;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.w = 1.0;

            broadcaster_->sendTransform(tf);
            return BT::NodeStatus::SUCCESS;
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_WARN(node_->get_logger(), "No box_frame: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }
    }

} // namespace warde_bt