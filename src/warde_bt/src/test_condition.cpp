#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>

#include "warde_bt/condition_box_present.h"
#include "warde_bt/condition_beer_present.h"
#include "warde_bt/action_get_closest_beer.h"
#include "warde_bt/action_navigate.h"
#include <robot_nav/srv/navigate.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_test");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    using NavigateReq = robot_nav::srv::Navigate::Request;
    using NavigateRes = robot_nav::srv::Navigate::Response;
    auto navigate_svc = node->create_service<robot_nav::srv::Navigate>(
        "navigate",
        [&](std::shared_ptr<NavigateReq> /*request*/,
            std::shared_ptr<NavigateRes> response)
        {
            response->success = true;
            response->message = "ok";
        });

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<warde_bt::ConditionBoxPresent>("ConditionBoxPresent");
    factory.registerNodeType<warde_bt::ConditionBeerPresent>("ConditionBeerPresent");
    factory.registerNodeType<warde_bt::ActionGetClosestBeer>("ActionGetClosestBeer");
    factory.registerNodeType<warde_bt::ActionNavigate>("ActionNavigate");

    auto pkg_share = ament_index_cpp::get_package_share_directory("warde_bt");
    auto xml_file = pkg_share + "/config/test_conditions.xml";
    RCLCPP_INFO(node->get_logger(), "Loading BT from: %s", xml_file.c_str());
    auto tree = factory.createTreeFromFile(xml_file);

    exec.spin_some();

    BT::TreeNode *root = tree.rootNode();
    RCLCPP_INFO(node->get_logger(), "Ticking full tree: Check→Beer→Pick→Navigate");
    BT::NodeStatus status = root->executeTick();

    RCLCPP_INFO(node->get_logger(),
                "Result of full Sequence: %s",
                status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");

    rclcpp::shutdown();
    return 0;
}
