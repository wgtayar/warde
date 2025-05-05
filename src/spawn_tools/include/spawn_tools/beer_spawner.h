#ifndef SPAWN_TOOLS__BEER_SPAWNER_HPP_
#define SPAWN_TOOLS__BEER_SPAWNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "spawn_tools/srv/spawn_beer.hpp"

namespace spawn_tools
{

    class BeerSpawner : public rclcpp::Node
    {
    public:
        BeerSpawner();

    private:
        void handle_spawn_beer(
            const std::shared_ptr<srv::SpawnBeer::Request> request,
            std::shared_ptr<srv::SpawnBeer::Response> response);

        rclcpp::CallbackGroup::SharedPtr client_cb_group_;
        rclcpp::CallbackGroup::SharedPtr service_cb_group_;
        rclcpp::Service<srv::SpawnBeer>::SharedPtr service_;
        rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr gazebo_client_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    };

} // namespace spawn_tools

#endif // SPAWN_TOOLS__BEER_SPAWNER_HPP_