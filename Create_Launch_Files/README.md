# Create Launch Files

Launch files in ROS 2 are used to start multiple nodes and configure their parameters with a single command. This README provides instructions on how to create launch files for your ROS 2 packages.

## Step 1: Create Package 

Use This sequence of commands to create a new package. Let's call our package my_robot_bringup.

```
cd ~/ros2_ws/src
ros2 pkg create my_robot_bringup 
cd ~/ros2_ws/
colcon build --packages-select my_robot_bringup
```

## Step 2: Create Launch Files

Use This sequence of commands to create Launch Files.Let's call our Node number_app.launch.py
```
cd ~/ros2_ws/src/my_robot_bringup/
rm -rf include/
rm -rf src
mkdir launch
cd launch
touch number_app.launch.py
chmod +x number_app.launch.py
```

add this template code for the python Node number_app.launch.py

```
from launch import LaunchDescription
from launch_ros.actions import Node

# Define a launch description that starts two nodes: a number publisher and a number counter
def generate_launch_description():
    ld = LaunchDescription()

    # Define a remapping for the number topic
    remap_number_topic = ("number", "my_number")

    # Create a node for publishing numbers
    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="my_number_publisher",
        remappings=[
            remap_number_topic
        ],
        parameters=[
            {"number_to_publish": 4},
            {"publish_frequency": 5.0}
        ]
    )

    # Create a node for counting numbers
    number_counter_node = Node(
        package="my_cpp_pkg",
        executable="number_counter",
        name="my_number_counter",
        remappings=[
            remap_number_topic,
            ("number_count", "my_number_count")
        ]
    )

    # Add both nodes to the launch description
    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)

    return ld
```

edit `package.xml` add this code 

```
   <exec_depend>my_py_pkg</exec_depend>
   <exec_depend>my_cpp_pkg</exec_depend>
```


Edit `CMakeLists.txt`

Replace your CMakeLists.txt code with the Following code

```
cmake_minimum_required(VERSION 3.5)
project(my_robot_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```


Build and run your c++ node: 

```
cd ~/ros2_ws/
source ~/.bashrc
colcon bulid --packages-select my_robot_bringup --symlink-install
ros2 launch my_robot_bringup number_app.launch.py
```
