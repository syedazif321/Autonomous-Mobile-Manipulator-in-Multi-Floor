#include "multi_floor_pipeline/ros2_bt_nodes.hpp"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <thread>
#include <chrono>

// Define the global ROS2 node
rclcpp::Node::SharedPtr g_ros_node;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_ros_node = rclcpp::Node::make_shared("multi_floor_pipeline_bt");

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ROS2ServiceNode>("ROS2Service");
    factory.registerNodeType<ROS2ActionNode>("ROS2Action");
    factory.registerNodeType<GoalNode>("Goal");
    factory.registerNodeType<PoseNode>("pose");
    factory.registerNodeType<PositionNode>("position");
    factory.registerNodeType<WaitNode>("Wait");

    std::string xml_file = ament_index_cpp::get_package_share_directory("multi_floor_pipeline") +
                           "/behavior_trees/pipeline.xml";

    RCLCPP_INFO(g_ros_node->get_logger(), "Loading Behavior Tree XML: %s", xml_file.c_str());

    BT::Tree tree = factory.createTreeFromFile(xml_file);

    while (rclcpp::ok() && tree.rootNode()->status() != BT::NodeStatus::SUCCESS)
    {
        tree.tickRoot();
        rclcpp::spin_some(g_ros_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(g_ros_node->get_logger(), "Behavior Tree execution finished.");
    rclcpp::shutdown();
    return 0;
}
