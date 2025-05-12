#include "spawn_tools/beer_spawner.h"
#include <fstream>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/callback_group.hpp>

using namespace std::chrono_literals;
using namespace spawn_tools;
using namespace std::placeholders;

BeerSpawner::BeerSpawner()
    : Node("beer_spawner")
{

    service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    service_ = this->create_service<srv::SpawnBeer>(
        "spawn_beer",
        std::bind(&BeerSpawner::handle_spawn_beer, this, _1, _2),
        rmw_qos_profile_services_default,
        service_cb_group_);

    gazebo_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>(
        "spawn_entity",
        rmw_qos_profile_services_default,
        client_cb_group_);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void BeerSpawner::handle_spawn_beer(
    const std::shared_ptr<srv::SpawnBeer::Request> request,
    std::shared_ptr<srv::SpawnBeer::Response> response)
{
    if (!gazebo_client_->wait_for_service(5s))
    {
        response->success = false;
        response->message = "spawn_entity service not available";
        return;
    }

    auto spawn_req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    spawn_req->name = request->entity_name;

    std::string pkg = ament_index_cpp::get_package_share_directory("spawn_tools");
    std::ifstream ifs(pkg + "/models/beer/model.sdf");
    std::stringstream ss;
    ss << ifs.rdbuf();
    spawn_req->xml = ss.str();

    spawn_req->initial_pose.position.x = request->x;
    spawn_req->initial_pose.position.y = request->y;
    spawn_req->initial_pose.position.z = request->z;
    spawn_req->reference_frame = "world";

    auto future = gazebo_client_->async_send_request(spawn_req);

    if (future.wait_for(10s) != std::future_status::ready)
    {
        response->success = false;
        response->message = "spawn_entity service timed out";
        return;
    }

    auto result = future.get();
    if (!result->success)
    {
        response->success = false;
        response->message = result->status_message;
        return;
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "odom";
    t.child_frame_id = request->entity_name + "_frame";
    t.transform.translation.x = request->x;
    t.transform.translation.y = request->y;
    t.transform.translation.z = request->z;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);

    response->success = true;
    response->message = "Beer spawned and TF published";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BeerSpawner>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
