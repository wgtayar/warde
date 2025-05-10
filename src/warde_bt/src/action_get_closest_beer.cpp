#include "warde_bt/action_get_closest_beer.h"
#include <rclcpp/clock.hpp>
#include <cmath>

namespace warde_bt
{

    ActionGetClosestBeer::ActionGetClosestBeer(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }

    BT::NodeStatus ActionGetClosestBeer::tick()
    {
        std::vector<std::string> beers;
        if (!getInput("beer_frames", beers))
        {
            throw BT::RuntimeError("ActionGetClosestBeer: missing required input port [beer_frames]");
        }
        if (beers.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("ActionGetClosestBeer"), "No beer frames found, beginning of the code.");
            return BT::NodeStatus::FAILURE;
        }

        double best_dist = std::numeric_limits<double>::infinity();
        std::string best_frame;

        for (auto &frame : beers)
        {
            try
            {
                // timeout 0.5s
                auto tf = buffer_->lookupTransform(
                    "chassis",
                    frame,
                    rclcpp::Time(0, 0, RCL_ROS_TIME),
                    rclcpp::Duration::from_seconds(0.5));

                double x = tf.transform.translation.x;
                double y = tf.transform.translation.y;
                double d = std::hypot(x, y);
                if (d < best_dist)
                {
                    best_dist = d;
                    best_frame = frame;
                }
            }
            catch (const tf2::TransformException &ex)
            {
                continue;
            }
        }

        if (best_frame.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("ActionGetClosestBeer"), "No beer frames found, end of the code.");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("target_beer_frame", best_frame);
        RCLCPP_INFO(rclcpp::get_logger("ActionGetClosestBeer"), "Closest beer frame: %s", best_frame.c_str());
        return BT::NodeStatus::SUCCESS;
    }
}
