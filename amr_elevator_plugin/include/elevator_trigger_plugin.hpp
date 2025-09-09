#ifndef ELEVATOR_TRIGGER_PLUGIN_HPP
#define ELEVATOR_TRIGGER_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>

namespace gazebo
{
  class ElevatorTriggerPlugin : public ModelPlugin
  {
  public:
    ElevatorTriggerPlugin() = default;
    virtual ~ElevatorTriggerPlugin() = default;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  private:
    void OnUpdate();
    void callElevatorService(bool state);

    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

    std::shared_ptr<rclcpp::Node> ros_node;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr elevator_client;

    // Target poses
    ignition::math::Vector3d target_pose_enter;
    ignition::math::Vector3d target_pose_exit;

    double tolerance;

    bool enter_called = false;
    bool exit_called = false;
  };
}

#endif
