
# 🤖 Autonomous Mobile Manipulator in Multi-Floor

<div align="center">
  <img src="gif/main_execution.gif" width="700">
</div>

## 🏗️ Overview

This repository contains a **complete ROS 2 Humble simulation** of an **autonomous mobile manipulator** capable of operating across **multiple floors** using an elevator system.  
It combines **navigation, manipulation, perception, and coordination** into one unified pipeline.

### 🧠 Key Components
- **AlphaBot Mobile Base (AMR)**
- **7-DOF RealMan Manipulator**
- **Linear Guide for Height Adjustment**
- **Gazebo Elevator Plugin** for multi-floor transport
- **Nav2** for navigation
- **MoveIt 2** for motion planning
- **Perception Pipeline** for pick-and-place operations
- **Database Logging** for task tracking

---

## 📁 Repository Structure

```bash
Autonomous-Mobile-Manipulator-in-Multi-Floor/
│
├── alphabot_description/           # Robot URDFs, meshes, and configs
├── rm_bringup/                     # RealMan manipulator bringup
├── custom_robots/                  # Multi-floor Gazebo world & robot spawn
├── mobile_manipulator_bringup/     # MoveIt 2 bringup configuration
├── alphabot_navigation/            # Nav2 navigation & Gazebo integration
├── pipeline_manipulator/           # FSM and perception pipeline
├── elevator_plugin/                # Gazebo elevator control plugin
├── gif/                            # Simulation demo GIFs
└── README.md
````

---

## ⚙️ Requirements

* **Ubuntu 22.04**
* **ROS 2 Humble**
* **Gazebo 11**
* **MoveIt 2**
* **Nav2**
* **teleop_twist_keyboard**
* **SQLite / MySQL** (for logging)

---

## 🧩 Environment Setup

Before launching, make sure Gazebo can locate models and plugins:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/install/alphabot_description/share
export GAZEBO_PLUGIN_PATH=$HOME/projetcs/Autonomous-Mobile-Manipulator-in-Multi-Floor/install/elevator_plugin/lib:$GAZEBO_PLUGIN_PATH
```

---

## 🚀 Full Simulation Launch (3-Terminal Setup)

Run these commands **in separate terminals** to start the full system.

### 🦾 Terminal 1 — MoveIt (Manipulator Control)

```bash
ros2 launch mobile_manipulator_bringup moveit_bringup.launch.py
```

### 🧭 Terminal 2 — Navigation + Gazebo

```bash
ros2 launch alphabot_navigation gazebo.launch.py
```

### 🧩 Terminal 3 — Pipeline / FSM (Main Logic)

```bash
ros2 run pipeline_manipulator pipeline_fsm
```

> 🧠 **Tip:** Launch in order — MoveIt → Navigation → Pipeline.

---

## 🎮 Optional Utilities

**Manual Teleoperation**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/amazon_robot/cmd_vel
```

**Elevator Control**

```bash
ros2 service call /elevator_cmd std_srvs/srv/SetBool "{data: true}"
```

**Linear Guide Control**

```bash
ros2 topic pub /slider_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]" -1
```

---

## 🧠 System Workflow

1. The **pipeline node** initializes and starts perception.
2. The **camera detects** the target package on the rack.
3. **Manipulator picks** the object using MoveIt.
4. **AMR navigates** to the elevator (Nav2).
5. The **elevator plugin** lifts the robot to the next floor.
6. **AMR drives** to the drop zone table.
7. **Arm places** the package and logs data into the database.

---

## 🧱 Core Features

✅ Multi-floor warehouse with elevator
✅ Integrated MoveIt 2 + Nav2 control
✅ Vision-based pick-and-place
✅ Linear guide for height reach
✅ Task database logging
✅ Dynamic obstacle simulation

---

## 📸 Simulation Demos

| Description      | Preview                                         |
| ---------------- | ----------------------------------------------- |
| Full System      | <img src="gif/main_execution.gif" width="500"/> |
| Warehouse World  | <img src="gif/world.gif" width="500"/>          |
| Linear Guide     | <img src="gif/linear_guide.gif" width="500"/>   |
| Database Logging | <img src="gif/data_base.gif" width="500"/>      |
| Pipeline Demo    | <img src="gif/pipeline1.gif" width="500"/>      |

---

## 🔧 Future Enhancements

* Add **3D vision-based grasping**
* Add **battery & docking simulation**
* Extend to **multi-robot elevator coordination**
* Real-world testing on physical robot setup

---

## 👨‍💻 Author

**Mohammed Azif**
📧 [syedazif321@gmail.com](mailto:syedazif321@gmail.com)
🔗 [GitHub Profile](https://github.com/syedazif321)

---

## 🧾 License

Released under the **MIT License**.

