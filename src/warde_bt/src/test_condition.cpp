#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "warde_bt/condition_box_present.h"
#include "warde_bt/condition_beer_present.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    // register both under warde_bt
    factory.registerNodeType<warde_bt::ConditionBoxPresent>("ConditionBoxPresent");
    factory.registerNodeType<warde_bt::ConditionBeerPresent>("ConditionBeerPresent");

    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("warde_bt");
    std::string xml_file = pkg_share + "/config/test_conditions.xml";
    RCLCPP_INFO(rclcpp::get_logger("bt_test"),
                "Loading test BT from: %s", xml_file.c_str());
    auto tree = factory.createTreeFromFile(xml_file);

    auto node = rclcpp::Node::make_shared("bt_condition_tester");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // one tick
    BT::NodeStatus status = tree.tickRoot();
    RCLCPP_INFO(node->get_logger(),
                "Result: %s",
                status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");

    rclcpp::shutdown();
    return 0;
}
