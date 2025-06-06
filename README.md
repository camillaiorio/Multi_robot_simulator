# Multi Robot Simulator

_Camilla Iorio_

## What?

This repository contains a simple multi-robot simulator for ROS, designed to simulate unicycle-style robots in a 2D environment. The simulator supports multiple robots, each with configurable parameters, and provides standard ROS topics for interaction.

## Features

### Map Loading

- Loads a `map.png` (grayscale PNG) and converts it into a `nav_msgs/OccupancyGrid`.
- Cells with pixel value < 127 are considered **occupied**; others are **free**.

### Robot Configuration Loader

Parses an XML file (`config.xml`) describing each robot’s:

- **Unique ID**
- **Maximum linear & angular velocities**
- **Radius** (for drawing/collision)
- **Initial pose** (`x`, `y`, `θ`)
- **LIDAR parameters** (number of beams, min/max range, frame_id)

Dynamically instantiates a `UnicyclePlatform` object for each robot.

### ROS Topic Publishers

For every robot `i` (`i=1…N`):

- **Odometry:** `robot_i/odom` (`nav_msgs/Odometry`)
- **PoseStamped:** `robot_i/robot_pose` (`geometry_msgs/PoseStamped`)
- **LaserScan:** `robot_i/scan` (`sensor_msgs/LaserScan`)
- **CmdVel:** `robot_i/cmd_vel` (`geometry_msgs/TwistStamped`)

**TF Transforms:**

- `map` to `robot_i` (robot’s base link)
- `robot_i` to `robot_i/laser` (laser frame; currently identity transform)

### OpenCV Visualization and Keyboard Control

The simulator uses OpenCV to visualize the environment and robot states, that permits real-time interaction. using the following controls:

- **Arrow keys:** Drive Robot 1
- **WASD:** Drive Robot 2
- **Spacebar:** Halt all robots

## How to compile

Tested on **Ubuntu 20.04** with **ROS Noetic**.

Install:

- ROS Noetic (`desktop-full` recommended)
- OpenCV (`libopencv-dev`)
- Eigen3 (`libeigen3-dev`)
- YAML-CPP (`libyaml-cpp-dev`)
- tinyxml2 (`libtinyxml2-dev`)
- Standard ROS build tools (`catkin`, `rosdep`, etc.)

From the root directory run the following command to compile the simulator:

```bash
./compile.bash
```

## Running the Simulator

1. **Start the ROS master:**

    ```bash
    ./run_server.bash
    ```

2. **Start the simulation:**

    ```bash
    ./run_node.bash
    ```

3. **Visualize the simulation in Rosviz:**

    ```bash
    ./run_rosviz.bash
    ```
