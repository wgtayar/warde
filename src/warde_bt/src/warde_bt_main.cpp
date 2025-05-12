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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("warde_bt_main");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT::RepeatNode>("DecoratorLoop");
    factory.registerNodeType<BT::ReactiveFallback>("FallbackNoMemory");
    factory.registerNodeType<warde_bt::ConditionBoxPresent>("ConditionBoxPresent");
    factory.registerNodeType<warde_bt::ConditionBeerPresent>("ConditionBeerPresent");
    factory.registerNodeType<warde_bt::ActionGetClosestBeer>("ActionGetClosestBeer");
    factory.registerNodeType<warde_bt::ActionNavigate>("ActionNavigate");
    factory.registerNodeType<warde_bt::ActionManipulate>("ActionManipulate");

    auto xml = ament_index_cpp::get_package_share_directory("warde_bt") + "/config/warde_tree.xml";
    auto tree = factory.createTreeFromFile(xml, blackboard);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
        status = tree.rootNode()->executeTick();
        exec.spin_some();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}