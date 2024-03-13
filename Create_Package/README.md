# Create Package

This guide walks you through Create Package. Follow the steps below:


## Step 1: Setting Up ROS2 Workspace 

Ensure that you have etting Up your Workspace. If not, follow these [steps](../Create_Workspace/README.md)


## Step 2: Create Package 

**python :** 

Use This sequence of commands to create a new package. Let's call our package my_py_package.

```
cd ~/ros2_ws/src
ros2 pkg create my_py_package --build-type ament_python --dependency rclpy
cd ~/ros2_ws/
colcon build --packages-select my_py_package
```

**C++ :** 

Use This sequence of commands to create a new package. Let's call our package my_cpp_package.

```
cd ~/ros2_ws/src
ros2 pkg create my_cpp_package --build-type ament_cmake --dependency rclcpp
cd ~/ros2_ws/
colcon bulid --packages-select my_cpp_package
```
