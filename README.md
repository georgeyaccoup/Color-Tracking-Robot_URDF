
# Color-Tracking-Robot_URDF

This repository contains the **URDF (Unified Robot Description Format)** implementation of a **color-tracking robot**.  
It provides the robot description (links, joints, sensors, and meshes) that can be visualized in **RViz** and simulated in **Gazebo**, forming the foundation for integrating vision-based object tracking with motion control.

---

##  Overview

The color-tracking robot is designed to detect and follow a colored object (e.g., a balloon) in its camera frame.  
This repository focuses on the **URDF model**, which defines:

- The robot’s **geometry and kinematics**  
- **Links** and **joints** representing the mechanical structure  
- **Inertial properties** for simulation stability  
- **Sensor frames** (e.g., for camera placement)  
- **Gazebo plugins and launch files** to run the robot in simulation  

This URDF is part of a broader system where ROS 2 nodes handle **color detection, kinematics, and PID control** for stepper motors.

---

## 📂 Repository Structure

```

├── config/              # Configuration files (e.g. sensors, controllers)
├── launch/              # ROS launch files to spawn robot in Gazebo / RViz
├── meshes/              # 3D mesh models of robot parts
├── resource/            # Auxiliary resources (textures, colors)
├── test/                # Example and test launch files
├── urdf/                # Main URDF description files
├── urdf_4/              # Alternate / experimental URDF version
├── world/               # Gazebo world files for simulation
├── package.xml          # ROS package manifest
└── README.md            # Project documentation

````

---

##  Implementation Process

### 1. Preparation (Meshes & CAD)
- Robot parts were modeled in CAD / 3D software.  
- Meshes exported as `.stl` or `.dae` and placed in the `meshes/` directory.  
- Simplified geometry for better simulation performance.

### 2. URDF Authoring
- Defined robot structure using `<link>` and `<joint>` elements.  
- Added `<visual>`, `<collision>`, and `<inertial>` properties.  
- Organized links for robot base, motors, and camera sensor.  

### 3. Sensors, Joints & Transmissions
- Joints specified with types (`revolute`, `continuous`, `fixed`).  
- Transmissions added for simulating actuators in Gazebo.  
- Camera frame defined for color-tracking functionality.  

### 4. Launch & Testing
- Launch files in `launch/` folder spawn the robot in **Gazebo** and **RViz**.  
- World files in `world/` provide test environments.  
- Verified alignment of links and frames using TF tree and RViz visualization.  

### 5. Iteration & Refinement
- Tuned inertial parameters for stable simulation.  
- Adjusted mesh scale and origins to match physical design.  
- Added simulation plugins for sensors and controllers.  

---

##  Usage

Clone the repository:
```bash
git clone https://github.com/georgeyaccoup/Color-Tracking-Robot_URDF.git
cd Color-Tracking-Robot_URDF
````

Build (ROS 2 example with `colcon`):

```bash
colcon build
source install/setup.bash
```

Launch in **Gazebo**:

```bash
ros2 launch <your_launch_file.launch.py>
```

Visualize in **RViz**:

```bash
ros2 launch <your_rviz_launch_file.launch.py>
```

---

##  Dependencies

* **ROS** (ROS 2 recommended, ROS 1 also supported)
* **Gazebo** with `gazebo_ros` package
* **RViz** for visualization
* `xacro` (if URDF is generated from `.xacro` files)
* Standard ROS packages: `tf2`, `sensor_msgs`, `geometry_msgs


