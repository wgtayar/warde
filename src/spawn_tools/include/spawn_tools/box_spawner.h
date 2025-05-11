#ifndef SPAWN_TOOLS__BOX_SPAWNER_HPP_
#define SPAWN_TOOLS__BOX_SPAWNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "spawn_tools/srv/spawn_box.hpp"

namespace spawn_tools
{

    class BoxSpawner : public rclcpp::Node
    {
    public:
        BoxSpawner();

    private:
        // callback for our custom SpawnBox service
        void handle_spawn_box(
            const std::shared_ptr<srv::SpawnBox::Request> request,
            std::shared_ptr<srv::SpawnBox::Response> response);

        rclcpp::CallbackGroup::SharedPtr client_cb_group_;
        rclcpp::CallbackGroup::SharedPtr service_cb_group_;
        rclcpp::Service<srv::SpawnBox>::SharedPtr service_;
        rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr gazebo_client_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    };

} // namespace spawn_tools

#endif // SPAWN_TOOLS__BOX_SPAWNER_HPP_