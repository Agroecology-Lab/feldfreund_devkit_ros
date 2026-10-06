# **Agro Robot Sim**

A ROS 2 Jazzy simulation package designed for the **Caatinga** agricultural robot. This module provides a complete Gazebo-based simulation environment, allowing for the testing of autonomous navigation and sensor integration in a risk-free virtual field.

## **Overview**

This package serves as the digital twin for the Caatinga robotics platform. It handles the transformation of Xacro/URDF files into a physical model within Gazebo and manages the robot's state via standard ROS 2 topics.

## **Package Structure**

The module is organized into the following key directories:

* **`launch/`**: Contains Python launch scripts for system initialization.  
  * `rsp.launch.py`: Processes the Xacro robot description and starts the `robot_state_publisher`.  
  * `sim.launch.py`: The primary entry point; launches Gazebo, the robot state, and spawns the robot.  
  * `spawn_robot.launch.py`: Utility script to inject the robot entity into a running simulation.  
* **`urdf/`**: Contains the `.xacro` files defining the robot's physical geometry, sensors, and Gazebo plugins.  
* **`worlds/`**: (Optional) Holds Gazebo world files representing agricultural environments.

## Headless simulation

Run `ros2 launch devkit_simulation sim.launch.py headless:=true` for software rendering on display `:99`.
This requires `Xvfb`, `xdpyinfo` (`x11-utils` on Ubuntu), and `timeout` (`coreutils`), included in the simulation image.
The launch reuses a display that accepts connections, or starts Xvfb and waits up to approximately 10 seconds for readiness before starting Gazebo.
World filenames containing spaces can be passed as a single quoted argument, such as `'world:=my field.world'`.

When `/dev/dri` exists and mount namespaces are permitted, the launch hides hardware render nodes from Gazebo in a private mount namespace.
Otherwise it runs without a mount namespace using Mesa software-rendering settings and warns if render nodes remain visible.
On boards whose Mesa EGL driver still selects a broken hardware render node, deploy without exposing `/dev/dri`, or explicitly allow mount namespaces and mounts (`CAP_SYS_ADMIN` plus the container's security policy).

## **Technical Specifications**

| Feature | Specification |
| :---- | :---- |
| **ROS Distribution** | ROS 2 Jazzy|
| **Build System** | `ament_cmake` |
| **Simulation Engine** | Gazebo |
| **Transform Tree** | Managed via `robot_state_publisher` |
| **License** | Apache-2.0 |
