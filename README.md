Absolutely! Let's create a comprehensive **README.md** for your ROS 2 project, covering the following components:

* **Gazebo Navigation** for mobile base control
* **MoveIt 2** for arm manipulation
* **Pipeline FSM** for orchestrating tasks

---

## 📚 Project Overview

This repository integrates:

* **Gazebo**: for 3D simulation of the robot's environment.
* **Navigation 2 (Nav2)**: for autonomous navigation and path planning.
* **MoveIt 2**: for motion planning and control of the robotic arm.
* **Pipeline FSM**: for managing the sequence of tasks in the manipulation pipeline.

---

## 🚀 Quick Start Guide

### 1. Launch Gazebo with Navigation 2

```bash
ros2 launch alphabot_navigation gazebo.launch.py
```

This command starts the Gazebo simulation with the Navigation 2 stack, enabling autonomous navigation capabilities for the robot.

### 2. Launch MoveIt 2 for Arm Control

```bash
ros2 launch mobile_manipulator_bringup moveit_bringup.launch.py
```

This command initializes MoveIt 2, allowing for motion planning and control of the robotic arm.

### 3. Run the Manipulation Pipeline

```bash
ros2 launch pipeline_manipulator pipeline_fsm.launch.py
```

This command starts the manipulation pipeline, coordinating tasks such as perception, planning, and execution.

---

## 🛠️ Setup Instructions

1. **Install Dependencies**

   Ensure you have ROS 2 installed. For ROS 2 Humble, follow the installation guide:

<<<<<<< HEAD
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

2. **Clone the Repository**

   Clone this repository into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/your-repo.git
   ```

3. **Install Required Packages**

   Navigate to your workspace and install dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the Workspace**

   Build the workspace using `colcon`:

   ```bash
   colcon build
   ```

5. **Source the Workspace**

   Source the workspace to overlay the new packages:

   ```bash
   source install/setup.bash
   ```

---

## 📄 File Structure

```
your-repo/
├── alphabot_navigation/
│   └── launch/
│       └── gazebo.launch.py
├── mobile_manipulator_bringup/
│   └── launch/
│       └── moveit_bringup.launch.py
└── pipeline_manipulator/
    └── launch/
        └── pipeline_fsm.launch.py
```

---

## 🧪 Testing the Setup

After sourcing your workspace, you can test individual components:

* **Gazebo with Navigation 2**:

  ```bash
  ros2 launch alphabot_navigation gazebo.launch.py
  ```

* **MoveIt 2 for Arm Control**:

  ```bash
  ros2 launch mobile_manipulator_bringup moveit_bringup.launch.py
  ```

* **Manipulation Pipeline**:

  ```bash
  ros2 launch pipeline_manipulator pipeline_fsm.launch.py
  ```

---

## 🧠 Troubleshooting Tips

* **Gazebo Launch Issues**: Ensure that all required plugins and models are correctly installed and sourced.
* **MoveIt 2 Configuration**: Verify that your URDF and SRDF files are correctly configured and accessible.
* **Pipeline FSM Errors**: Check the logs for any missing dependencies or misconfigurations in the launch files.

---

## 🔗 Useful Resources

* [ROS 2 Navigation 2 Documentation](https://docs.ros.org/en/foxy/Tutorials/Navigation2/Navigation2-Tutorial.html)
* [MoveIt 2 Documentation](https://moveit.picknik.ai/main/doc/index.html)
* [Gazebo ROS 2 Integration](https://gazebosim.org/docs/latest/ros2_integration/)

---

Feel free to customize this README further based on your specific project details and requirements.
=======
ros2 topic pub /rm_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp:
    sec: 0
    nanosec: 0
joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  - joint7
points:
  - positions: [-1.63441, 0.21853, 0.03951, 1.14246, 0.12855, 0.16098, -0.24668]
    time_from_start:
      sec: 3
      nanosec: 0
"

ros2 service call /switch_floor std_srvs/srv/SetBool "{data: true}"
>>>>>>> origin/long_distance
