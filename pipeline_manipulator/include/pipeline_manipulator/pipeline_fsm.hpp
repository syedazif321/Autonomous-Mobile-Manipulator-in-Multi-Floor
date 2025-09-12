#ifndef PIPELINE_MANIPULATOR_PIPELINE_FSM_HPP
#define PIPELINE_MANIPULATOR_PIPELINE_FSM_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <msg_gazebo/srv/attach_detach.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class PipelineFSM : public rclcpp::Node
{
public:
    enum class State {
        IDLE,
        NAV_TO_PICK_PRE,
        WAIT_NAV_PICK_PRE,
        SLIDER_EXTEND,
        SPAWN_BOX,
        WAIT_AFTER_SPAWN,
        MOVE_ARM_SAFE_PICK,
        START_VISION,
        WAIT_VISION_COMPLETE,
        START_PICKING,
        ATTACH_OBJECT,
        MOVE_ARM_HOME,
        NAV_TO_MIDDLE,
        WAIT_NAV_MIDDLE,
        NAV_TO_LIFT_0,
        WAIT_NAV_LIFT_0,
        CALL_ELEVATOR,
        SWITCH_FLOOR,
        NAV_TO_DROP_PRE_AMR,
        WAIT_NAV_DROP_PRE_AMR,
        MOVE_ARM_DROP_PRE,
        MOVE_ARM_DROP,
        DETACH_OBJECT,
        MOVE_ARM_HOME_FINAL,
        DONE
    };

    PipelineFSM();

private:
    void runFSM();
    void transitionTo(State next); 
    void loadTargets();
    void navigateTo(const std::string& target_name);
    void sendSliderTrajectory();
    void callTriggerService(const std::string& service_name);
    void callStartDetection();
    void callStartPicking();
    void callAttachDetach(bool attach);
    void callSetBoolService(const std::string& service_name, bool data);
    void publishBoolTopic(const std::string& topic_name, bool data);
    void sendArmTrajectory(const std::string& joint_target_name);

    State current_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr delay_timer_;

    // Action clients
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr slider_client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

    // Service clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr spawn_box_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_picking_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_detection_client_;
    rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_detach_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr elevator_client_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr floor_pub_;

    bool navigation_goal_sent_ = false;
    bool navigation_complete_ = false;
    bool navigation_succeeded_ = false;

    bool slider_goal_active_ = false;


    bool arm_goal_active_ = false;

    // Loaded targets
    nlohmann::json targets_json_;

    // Delays (in ms)
    const int DELAY_500MS = 500;
    const int DELAY_1000MS = 1000;
    const int DELAY_2000MS = 2000;
    const int DELAY_3000MS = 3000;
    const int DELAY_200MS = 200;
};

#endif // PIPELINE_MANIPULATOR_PIPELINE_FSM_HPP