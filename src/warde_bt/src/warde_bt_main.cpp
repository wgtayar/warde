#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <behaviortree_cpp_v3/decorators/repeat_node.h>
#include <behaviortree_cpp_v3/controls/reactive_sequence.h>
#include <behaviortree_cpp_v3/controls/reactive_fallback.h>

#include "warde_bt/condition_box_present.h"
#include "warde_bt/condition_beer_present.h"
#include "warde_bt/action_get_closest_beer.h"
#include "warde_bt/action_navigate.h"
#include "warde_bt/action_manipulate.h"
#include "robot_nav/srv/navigate.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("warde_bt_main");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    using NavigateReq = robot_nav::srv::Navigate::Request;
    using NavigateRes = robot_nav::srv::Navigate::Response;
    node->create_service<robot_nav::srv::Navigate>(
        "navigate",
        [&](std::shared_ptr<NavigateReq>, std::shared_ptr<NavigateRes> res)
        {
            res->success = true;
            res->message = "ok";
        });

    auto bb = BT::Blackboard::create();
    bb->set("ros_node", node);

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<BT::RepeatNode>("DecoratorLoop");
    factory.registerNodeType<BT::ReactiveFallback>("FallbackNoMemory");
    factory.registerNodeType<warde_bt::ConditionBoxPresent>("ConditionBoxPresent");
    factory.registerNodeType<warde_bt::ConditionBeerPresent>("ConditionBeerPresent");
    factory.registerNodeType<warde_bt::ActionGetClosestBeer>("ActionGetClosestBeer");
    factory.registerNodeType<warde_bt::ActionNavigate>("ActionNavigate");
    factory.registerNodeType<warde_bt::ActionManipulate>("ActionManipulate");

    auto pkg_share = ament_index_cpp::get_package_share_directory("warde_bt");
    auto xml_file = pkg_share + "/config/warde_tree.xml";
    RCLCPP_INFO(node->get_logger(), "Loading BT from: %s", xml_file.c_str());

    factory.registerBehaviorTreeFromFile(xml_file);
    auto tree = factory.createTree("MainTree", bb);

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok())
    {
        tree.rootNode()->executeTick();
        exec.spin_some();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}