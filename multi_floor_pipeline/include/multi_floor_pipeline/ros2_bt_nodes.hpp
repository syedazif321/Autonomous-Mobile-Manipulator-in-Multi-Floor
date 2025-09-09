#pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <chrono>

// Global ROS2 node
extern rclcpp::Node::SharedPtr g_ros_node;

// ------------------ ROS2 Service Node ------------------
class ROS2ServiceNode : public BT::SyncActionNode
{
public:
    ROS2ServiceNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), done_(false) {}

    BT::NodeStatus tick() override
    {
        if (done_) return BT::NodeStatus::SUCCESS;  // call once

        if(!g_ros_node) {
            RCLCPP_ERROR(rclcpp::get_logger("ROS2ServiceNode"), "ROS2 Node not initialized!");
            return BT::NodeStatus::FAILURE;
        }

        auto service_name = getInput<std::string>("service_name").value_or("");
        if (service_name.empty()) return BT::NodeStatus::FAILURE;

        auto client = g_ros_node->create_client<std_srvs::srv::Trigger>(service_name);
        if (!client->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(g_ros_node->get_logger(), "Service %s not available", service_name.c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = client->async_send_request(req);
        rclcpp::spin_until_future_complete(g_ros_node, result);

        done_ = true;
        return result.get()->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("service_name") };
    }

private:
    bool done_;
};

// ------------------ ROS2 Action Node ------------------
class ROS2ActionNode : public BT::SyncActionNode
{
public:
    ROS2ActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), done_(false) {}

    BT::NodeStatus tick() override
    {
        if (done_) return BT::NodeStatus::SUCCESS;

        auto action_name = getInput<std::string>("action_name").value_or("");
        auto goal_msg = getInput<std::string>("goal_msg").value_or("");

        RCLCPP_INFO(g_ros_node->get_logger(), "Would send goal to action: %s, goal_msg: %s",
                    action_name.c_str(), goal_msg.c_str());

        // TODO: Implement actual ROS2 action client call

        done_ = true;
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("action_name"),
                 BT::InputPort<std::string>("goal_msg") };
    }

private:
    bool done_;
};

// ------------------ Goal Node ------------------
class GoalNode : public BT::SyncActionNode
{
public:
    GoalNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), done_(false) {}

    BT::NodeStatus tick() override
    {
        if (done_) return BT::NodeStatus::SUCCESS;

        auto pose = getInput<std::string>("pose").value_or("");
        RCLCPP_INFO(g_ros_node->get_logger(), "GoalNode executing, pose: %s", pose.c_str());

        // TODO: Send Nav2 or MoveIt goal

        done_ = true;
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("pose") };
    }

private:
    bool done_;
};

// ------------------ Pose Node ------------------
class PoseNode : public BT::SyncActionNode
{
public:
    PoseNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), done_(false) {}

    BT::NodeStatus tick() override
    {
        if (done_) return BT::NodeStatus::SUCCESS;

        auto pose_name = getInput<std::string>("pose_name").value_or("undefined");
        RCLCPP_INFO(g_ros_node->get_logger(), "PoseNode: Moving to pose %s", pose_name.c_str());

        // TODO: MoveIt/Nav2 call here

        done_ = true;
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("pose_name") };
    }

private:
    bool done_;
};

// ------------------ Position Node ------------------
class PositionNode : public BT::SyncActionNode
{
public:
    PositionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), done_(false) {}

    BT::NodeStatus tick() override
    {
        if (done_) return BT::NodeStatus::SUCCESS;

        auto pos_name = getInput<std::string>("position_name").value_or("undefined");
        RCLCPP_INFO(g_ros_node->get_logger(), "PositionNode: Moving to position %s", pos_name.c_str());

        // TODO: Send joint trajectory

        done_ = true;
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("position_name") };
    }

private:
    bool done_;
};

// ------------------ Wait Node ------------------
class WaitNode : public BT::SyncActionNode
{
public:
    WaitNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override
    {
        int msec = getInput<int>("msec").value_or(1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(msec));
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("msec") };
    }
};
