#include "elevator_trigger_plugin.hpp"
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  void ElevatorTriggerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model = _model;

    // ROS2 node init
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    ros_node = std::make_shared<rclcpp::Node>("elevator_trigger_plugin");
    elevator_client = ros_node->create_client<std_srvs::srv::SetBool>("/elevator_cmd");
      gzlog << "[ElevatorTriggerPlugin] Successfully loaded!" << std::endl;

    // If attached to a specific robot
    gzlog << "[ElevatorTriggerPlugin] Model Name: " << _model->GetName() << std::endl;

    // Load parameters from SDF
    if (_sdf->HasElement("target_pose_enter"))
    {
      auto elem = _sdf->GetElement("target_pose_enter");
      target_pose_enter = ignition::math::Vector3d(
        elem->Get<double>("x"),
        elem->Get<double>("y"),
        elem->Get<double>("z"));
    }
    else
    {
      gzerr << "[ElevatorTriggerPlugin] Missing <target_pose_enter> in SDF.\n";
      return;
    }

    if (_sdf->HasElement("target_pose_exit"))
    {
      auto elem = _sdf->GetElement("target_pose_exit");
      target_pose_exit = ignition::math::Vector3d(
        elem->Get<double>("x"),
        elem->Get<double>("y"),
        elem->Get<double>("z"));
    }
    else
    {
      gzerr << "[ElevatorTriggerPlugin] Missing <target_pose_exit> in SDF.\n";
      return;
    }

    tolerance = _sdf->HasElement("tolerance") ? _sdf->Get<double>("tolerance") : 0.5;

    updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ElevatorTriggerPlugin::OnUpdate, this));

    gzlog << "[ElevatorTriggerPlugin] Plugin loaded successfully.\n";
  }

  void ElevatorTriggerPlugin::OnUpdate()
  {
    if (!ros_node) return;

    auto pose = model->WorldPose().Pos();

    // Check for enter position
    if (!enter_called && pose.Distance(target_pose_enter) <= tolerance)
    {
      RCLCPP_INFO(ros_node->get_logger(), "Robot reached ENTER target. Calling elevator open service...");
      callElevatorService(true);
      enter_called = true;
    }

    // Check for exit position
    if (!exit_called && pose.Distance(target_pose_exit) <= tolerance)
    {
      RCLCPP_INFO(ros_node->get_logger(), "Robot reached EXIT target. Calling elevator close service...");
      callElevatorService(false);
      exit_called = true;
    }

    rclcpp::spin_some(ros_node);
  }

  void ElevatorTriggerPlugin::callElevatorService(bool state)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = state;

    if (!elevator_client->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(ros_node->get_logger(), "Elevator service not available!");
      return;
    }

    auto result = elevator_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(ros_node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(ros_node->get_logger(), "Elevator service called successfully!");
    }
    else
    {
      RCLCPP_ERROR(ros_node->get_logger(), "Failed to call elevator service.");
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(ElevatorTriggerPlugin)
}
