#include "pipeline_manipulator/pipeline_fsm.hpp"
#include <unistd.h>
#include <limits.h>

PipelineFSM::PipelineFSM()
: Node("pipeline_fsm_node"), current_state_(State::IDLE)
{
    // Initialize action clients
    slider_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, "/slider_position_controller/follow_joint_trajectory");

    arm_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this, "/rm_group_controller/follow_joint_trajectory");

    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    // Initialize service clients
    spawn_box_client_ = this->create_client<std_srvs::srv::Trigger>("/spawn_box");
    start_picking_client_ = this->create_client<std_srvs::srv::Trigger>("/start_picking");
    start_detection_client_ = this->create_client<std_srvs::srv::Trigger>("/start_detection"); // 👈 NEW
    attach_detach_client_ = this->create_client<msg_gazebo::srv::AttachDetach>("/AttachDetach");
    elevator_client_ = this->create_client<std_srvs::srv::SetBool>("/elevator_cmd");

    // Initialize publisher
    floor_pub_ = this->create_publisher<std_msgs::msg::Bool>("/use_floor_1", 10);

    // Wait for servers
    while (!slider_client_->wait_for_action_server(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for slider trajectory server...");
    }
    while (!arm_client_->wait_for_action_server(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for arm trajectory server...");
    }
    while (!nav2_client_->wait_for_action_server(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 /navigate_to_pose action server...");
    }
    while (!spawn_box_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /spawn_box service...");
    }
    while (!start_picking_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /start_picking service...");
    }
    while (!start_detection_client_->wait_for_service(1s) && rclcpp::ok()) { // 👈 NEW
        RCLCPP_INFO(this->get_logger(), "Waiting for /start_detection service...");
    }
    while (!attach_detach_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /AttachDetach service...");
    }
    while (!elevator_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /elevator_cmd service...");
    }

    // Load targets
    loadTargets();

    // Start FSM
    timer_ = this->create_wall_timer(200ms, std::bind(&PipelineFSM::runFSM, this));
    RCLCPP_INFO(this->get_logger(), "Pipeline FSM started.");
}

void PipelineFSM::loadTargets()
{
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len != -1) {
        exe_path[len] = '\0';
        std::string exec_dir(exe_path);
        size_t last_slash = exec_dir.find_last_of("/");
        if (last_slash != std::string::npos) {
            exec_dir = exec_dir.substr(0, last_slash);
        }
        std::string json_path = exec_dir + "/../../robot_data/Targets.json";
        std::ifstream f(json_path);
        if (!f.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Targets.json at %s", json_path.c_str());
            throw std::runtime_error("JSON file not found");
        }
        try {
            targets_json_ = nlohmann::json::parse(f);
            RCLCPP_INFO(this->get_logger(), "Targets loaded from %s", json_path.c_str());
        } catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
            throw std::runtime_error("Failed to parse Targets.json");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not resolve executable path");
        throw std::runtime_error("Cannot resolve path");
    }
}

void PipelineFSM::navigateTo(const std::string& target_name)
{
    if (!targets_json_["amr_poses"].contains(target_name)) {
        RCLCPP_ERROR(this->get_logger(), "AMR pose '%s' not found in JSON", target_name.c_str());
        return;
    }

    auto& pose_data = targets_json_["amr_poses"][target_name];
    nav2_msgs::action::NavigateToPose::Goal goal;

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();

    goal.pose.pose.position.x = pose_data["position"][0];
    goal.pose.pose.position.y = pose_data["position"][1];
    goal.pose.pose.position.z = pose_data["position"][2];

    goal.pose.pose.orientation.x = pose_data["orientation"][0];
    goal.pose.pose.orientation.y = pose_data["orientation"][1];
    goal.pose.pose.orientation.z = pose_data["orientation"][2];
    goal.pose.pose.orientation.w = pose_data["orientation"][3];

    // Reset flags
    navigation_complete_ = false;
    navigation_succeeded_ = false;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Navigation goal REJECTED");
                navigation_complete_ = true;
                navigation_succeeded_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Navigation goal ACCEPTED");
            }
        };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
            navigation_complete_ = true;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Navigation SUCCEEDED");
                    navigation_succeeded_ = true;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Navigation ABORTED");
                    navigation_succeeded_ = false;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Navigation CANCELED");
                    navigation_succeeded_ = false;
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Navigation UNKNOWN result");
                    navigation_succeeded_ = false;
                    break;
            }
        };

    nav2_client_->async_send_goal(goal, send_goal_options);
    navigation_goal_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "Navigation goal sent to '%s'", target_name.c_str());
}

void PipelineFSM::sendSliderTrajectory()
{
    if (slider_goal_active_) {
        RCLCPP_WARN(this->get_logger(), "Slider goal already active — skipping");
        return;
    }

    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"slider_joint"};
    goal_msg.trajectory.points.resize(2);

    goal_msg.trajectory.points[0].positions = {0.0};
    goal_msg.trajectory.points[0].velocities = {0.0};
    goal_msg.trajectory.points[0].accelerations = {0.0};
    goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(1s);

    goal_msg.trajectory.points[1].positions = {0.5};
    goal_msg.trajectory.points[1].velocities = {0.0};
    goal_msg.trajectory.points[1].accelerations = {0.0};
    goal_msg.trajectory.points[1].time_from_start = rclcpp::Duration(6s);

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Slider goal rejected");
                slider_goal_active_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Slider goal accepted");
            }
        };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
            slider_goal_active_ = false;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "✅ Slider trajectory succeeded");
                this->transitionTo(State::SPAWN_BOX);
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ Slider trajectory failed with code: %d", static_cast<int>(result.code));
                this->transitionTo(State::SPAWN_BOX);
            }
        };

    slider_client_->async_send_goal(goal_msg, send_goal_options);
    slider_goal_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Sent slider trajectory goal");
}

void PipelineFSM::callTriggerService(const std::string& service_name)
{
    auto client = (service_name == "/spawn_box") ? spawn_box_client_ : start_picking_client_;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    client->async_send_request(request,
        [this, service_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "%s succeeded", service_name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "%s failed: %s", service_name.c_str(), response->message.c_str());
            }
        });
}

void PipelineFSM::callStartDetection()  // 👈 NEW FUNCTION
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    start_detection_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✅ Vision detection STARTED");
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ Failed to start vision detection: %s", response->message.c_str());
            }
        });
}

void PipelineFSM::callAttachDetach(bool attach)
{
    auto request = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
    request->model1 = "mobile_manipulator";
    request->link1 = "Link7";
    request->model2 = "model_0";
    request->link2 = "link";
    request->attach = attach;

    attach_detach_client_->async_send_request(request,
        [this, attach](rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✅ Attach/Detach %s succeeded", attach ? "ATTACH" : "DETACH");
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ Attach/Detach failed");
            }
        });
}

void PipelineFSM::callSetBoolService(const std::string& service_name, bool data)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = data;

    elevator_client_->async_send_request(request,
        [this, service_name, data](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✅ %s set to %s", service_name.c_str(), data ? "true" : "false");
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ %s failed: %s", service_name.c_str(), response->message.c_str());
            }
        });
}

void PipelineFSM::publishBoolTopic(const std::string& topic_name, bool data)
{
    auto msg = std_msgs::msg::Bool();
    msg.data = data;
    floor_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "✅ Published %s to %s", data ? "true" : "false", topic_name.c_str());
}

void PipelineFSM::sendArmTrajectory(const std::string& joint_target_name)
{
    if (!targets_json_["arm_positions"].contains(joint_target_name)) {
        RCLCPP_ERROR(this->get_logger(), "Arm position '%s' not found", joint_target_name.c_str());
        return;
    }

    auto& joint_values = targets_json_["arm_positions"][joint_target_name];
    std::vector<double> positions;
    for (auto& v : joint_values) {
        positions.push_back(v.get<double>());
    }

    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    goal_msg.trajectory.points.resize(1);

    goal_msg.trajectory.points[0].positions = positions;
    goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(3s);

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this, joint_target_name](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Arm goal '%s' rejected", joint_target_name.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Arm goal '%s' accepted", joint_target_name.c_str());
            }
        };

    send_goal_options.result_callback =
        [this, joint_target_name](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "✅ Arm movement to '%s' succeeded", joint_target_name.c_str());
                switch (this->current_state_) {
                    case State::MOVE_ARM_SAFE_PICK:
                        this->transitionTo(State::START_PICKING_1);
                        break;
                    case State::MOVE_ARM_HOME:
                        this->transitionTo(State::NAV_TO_MIDDLE);
                        break;
                    case State::MOVE_ARM_DROP_PRE:
                        this->transitionTo(State::MOVE_ARM_DROP);
                        break;
                    case State::MOVE_ARM_DROP:
                        this->transitionTo(State::DETACH_OBJECT);
                        break;
                    case State::MOVE_ARM_HOME_FINAL:
                        this->transitionTo(State::DONE);
                        break;
                    default:
                        break;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ Arm movement to '%s' failed", joint_target_name.c_str());
            }
        };

    arm_client_->async_send_goal(goal_msg, send_goal_options);
}

void PipelineFSM::transitionTo(State next)
{
    RCLCPP_INFO(this->get_logger(), "🔄 TRANSITION: %d → %d", static_cast<int>(current_state_), static_cast<int>(next));
    current_state_ = next;
}

void PipelineFSM::runFSM()
{
    switch (current_state_) {
        case State::IDLE:
            transitionTo(State::NAV_TO_PICK_PRE);
            break;

        case State::NAV_TO_PICK_PRE:
            navigateTo("pick_pre_amr");
            transitionTo(State::WAIT_NAV_PICK_PRE);
            break;

        case State::WAIT_NAV_PICK_PRE:
            if (!navigation_complete_) return;
            if (!navigation_succeeded_) {
                RCLCPP_ERROR(this->get_logger(), "❌ Failed to reach pick_pre_amr. Retrying...");
                transitionTo(State::NAV_TO_PICK_PRE);
                return;
            }
            transitionTo(State::SLIDER_EXTEND);
            break;

        case State::SLIDER_EXTEND:
            sendSliderTrajectory();
            break;

        case State::SPAWN_BOX:
            callTriggerService("/spawn_box");
            transitionTo(State::START_VISION);
            break;

        case State::START_VISION:
            callStartDetection();
            timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_2000MS), // 2 seconds
                [this]() {
                    this->transitionTo(State::MOVE_ARM_SAFE_PICK);
                    this->timer_->cancel();
                });
            break;

        case State::MOVE_ARM_SAFE_PICK:
            sendArmTrajectory("safe_pick_arm");
            break;

        case State::START_PICKING_1:
            callTriggerService("/start_picking");
            timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_300MS),
                [this]() {
                    this->callTriggerService("/start_picking");
                    this->transitionTo(State::ATTACH_OBJECT);
                    this->timer_->cancel();
                });
            break;

        case State::ATTACH_OBJECT:
            callAttachDetach(true);
            transitionTo(State::MOVE_ARM_HOME);
            break;

        case State::MOVE_ARM_HOME:
            sendArmTrajectory("home");
            break;

        case State::NAV_TO_MIDDLE:
            navigateTo("middle_amr");
            transitionTo(State::WAIT_NAV_MIDDLE);
            break;

        case State::WAIT_NAV_MIDDLE:
            if (!navigation_complete_) return;
            if (!navigation_succeeded_) {
                RCLCPP_ERROR(this->get_logger(), "❌ Failed to reach middle_amr");
                return;
            }
            transitionTo(State::NAV_TO_LIFT_0);
            break;

        case State::NAV_TO_LIFT_0:
            navigateTo("lift_0_amr");
            transitionTo(State::WAIT_NAV_LIFT_0);
            break;

        case State::WAIT_NAV_LIFT_0:
            if (!navigation_complete_) return;
            if (!navigation_succeeded_) {
                RCLCPP_ERROR(this->get_logger(), "❌ Failed to reach lift_0_amr");
                return;
            }
            transitionTo(State::CALL_ELEVATOR);
            break;

        case State::CALL_ELEVATOR:
            callSetBoolService("/elevator_cmd", true);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_2000MS),
                [this]() {
                    this->transitionTo(State::SWITCH_FLOOR);
                    this->timer_->cancel();
                });
            break;

        case State::SWITCH_FLOOR:
            publishBoolTopic("/use_floor_1", true);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_1000MS),
                [this]() {
                    this->transitionTo(State::NAV_TO_DROP_PRE_AMR);
                    this->timer_->cancel();
                });
            break;

        case State::NAV_TO_DROP_PRE_AMR:
            navigateTo("drop_pre_amr");
            transitionTo(State::WAIT_NAV_DROP_PRE_AMR);
            break;

        case State::WAIT_NAV_DROP_PRE_AMR:
            if (!navigation_complete_) return;
            if (!navigation_succeeded_) {
                RCLCPP_ERROR(this->get_logger(), "❌ Failed to reach drop_pre_amr");
                return;
            }
            transitionTo(State::MOVE_ARM_DROP_PRE);
            break;

        case State::MOVE_ARM_DROP_PRE:
            sendArmTrajectory("drop_pre");
            break;

        case State::MOVE_ARM_DROP:
            sendArmTrajectory("drop");
            break;

        case State::DETACH_OBJECT:
            callAttachDetach(false);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(DELAY_500MS),
                [this]() {
                    this->transitionTo(State::MOVE_ARM_HOME_FINAL);
                    this->timer_->cancel();
                });
            break;

        case State::MOVE_ARM_HOME_FINAL:
            sendArmTrajectory("home");
            break;

        case State::DONE:
            RCLCPP_INFO(this->get_logger(), "🎉✅ PIPELINE COMPLETED SUCCESSFULLY");
            timer_->cancel();
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "⚠️ Unknown state");
            break;
    }
}