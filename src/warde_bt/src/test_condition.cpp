#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include "warde_bt/condition_box_present.h"
#include "warde_bt/condition_beer_present.h"
#include "warde_bt/action_get_closest_beer.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<warde_bt::ConditionBoxPresent>("ConditionBoxPresent");
    factory.registerNodeType<warde_bt::ConditionBeerPresent>("ConditionBeerPresent");
    factory.registerNodeType<warde_bt::ActionGetClosestBeer>("ActionGetClosestBeer");

    auto pkg_share = ament_index_cpp::get_package_share_directory("warde_bt");
    auto xml_file = pkg_share + "/config/test_conditions.xml";
    RCLCPP_INFO(rclcpp::get_logger("bt_test"), "Loading BT from: %s", xml_file.c_str());
    auto tree = factory.createTreeFromFile(xml_file);

    auto node = rclcpp::Node::make_shared("bt_condition_tester");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    auto &all_nodes = tree.nodes;

    auto find_node = [&](const std::string &nm) -> std::shared_ptr<BT::TreeNode>
    {
        auto it = std::find_if(
            all_nodes.begin(), all_nodes.end(),
            [&](auto &sp)
            { return sp->name() == nm; });
        if (it == all_nodes.end())
        {
            RCLCPP_ERROR(node->get_logger(), "Node '%s' not found in tree!", nm.c_str());
            std::exit(1);
        }
        return *it;
    };

    // NOTE
    // We need to publish the TF frame transformation from world to chassis before being able to find the relative distance between the chassis
    // and the beer frames. 
    // LOOK INTO THE TF2 STATIC FRAME BROADCASTER AND IF WE NEED TO GO FOR THE DYNAMIC ONE

    auto box_node = find_node("BoxPresent");
    auto beer_node = find_node("BeerAvail");
    auto action_node = find_node("ClosestBeer");

    RCLCPP_INFO(node->get_logger(), "Ticking ConditionBoxPresent");
    auto s1 = box_node->executeTick();
    RCLCPP_INFO(node->get_logger(),
                "    -> ConditionBoxPresent: %s",
                s1 == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
    if (s1 != BT::NodeStatus::SUCCESS)
    {
        RCLCPP_WARN(node->get_logger(), "Stopping early: no box!");
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Ticking ConditionBeerPresent");
    auto s2 = beer_node->executeTick();
    RCLCPP_INFO(node->get_logger(),
                "    -> ConditionBeerPresent: %s",
                s2 == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
    if (s2 != BT::NodeStatus::SUCCESS)
    {
        RCLCPP_WARN(node->get_logger(), "Stopping early: no beers!");
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Ticking ActionGetClosestBeer");
    auto s3 = action_node->executeTick();
    RCLCPP_INFO(node->get_logger(),
                "    -> ActionGetClosestBeer: %s",
                s3 == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");

    if (s3 == BT::NodeStatus::SUCCESS)
    {
        std::string closest;
        if (tree.rootBlackboard()->get("target_beer_frame", closest))
        {
            RCLCPP_INFO(node->get_logger(),
                        "Closest beer_frame is: %s", closest.c_str());
        }
        else
        {
            RCLCPP_WARN(node->get_logger(),
                        "Succeeded but no 'target_beer_frame' on blackboard");
        }
    }

    rclcpp::shutdown();
    return 0;
}
