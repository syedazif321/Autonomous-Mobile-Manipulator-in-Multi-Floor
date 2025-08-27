#ifndef GAZEBO_PLUGINS_ELEVATORPLUGIN_HH_
#define GAZEBO_PLUGINS_ELEVATORPLUGIN_HH_

#include <memory>
#include <string>

#include <sdf/sdf.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

#include <std_srvs/srv/set_bool.hpp>
#include <gazebo_ros/node.hpp>

namespace gazebo
{
  class GZ_PLUGIN_VISIBLE ElevatorPlugin : public ModelPlugin
  {
    public: ElevatorPlugin();
    public: ~ElevatorPlugin();

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    public: void Reset() override;

    /// Move to a particular floor
    public: void MoveToFloor(const int _floor);

    private: void Update(const common::UpdateInfo &_info);
    private: void OnElevator(ConstGzStringPtr &_msg);

    /// ROS2 service
    private: void RosServiceCb(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // ROS2 members (gazebo_ros node handle)
    private: gazebo_ros::Node::SharedPtr ros_node_;
    private: rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
    private: int current_floor_{0};

    // Internal private impl
    private: class ElevatorPluginPrivate;
    private: std::unique_ptr<ElevatorPluginPrivate> dataPtr;
  };
}
#endif
