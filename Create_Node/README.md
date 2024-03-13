# Create Node

This guide walks you through Create Node. Follow the steps below:

## Step 1: Create Package 

Ensure that you have created your package. If not, follow these [steps](../Create_Package/README.md)

## Step 2: Create Node 

**python :**

Use This sequence of commands to create a new Node. Let's call our Node first_py_node.
```
cd ~/ros2_ws/src/my_py_package/my_py_package
touch first_py_node.py
chmod +x first_py_node.py
cd ~/ros2_ws/
colcon bulid --packages-select my_py_package
cd ~/ros2_ws/src/my_py_package/my_py_package
./first_py_node.py
```

add this template code for the python Node first_py_node.py

```
# Import the rclpy library, which provides the Python API for ROS 2
import rclpy
# Import the Node class from rclpy.node module, which is the base class for all ROS 2 nodes
from rclpy.node import Node

# Define a custom node class that inherits from the Node class
class MyCustomNode(Node):
    # Constructor method for the custom node class
    def __init__(self):
        # Call the constructor of the parent class with the name of the node ("node_name")
        super().__init__("node_name")
        # Print "Hello, World!" when the node is initialized
        self.get_logger().info("Hello, World!")

# Main function to initialize and run the node
def main(args=None):
    # Initialize the ROS 2 framework, passing any command-line arguments
    rclpy.init(args=args)
    # Create an instance of the custom node class
    node = MyCustomNode()
    # Enter the ROS 2 event loop, spinning the node to process messages
    rclpy.spin(node)
    # Shutdown the ROS 2 framework after exiting the event loop
    rclpy.shutdown()

# Entry point of the script
if __name__ == "__main__":
    # Call the main function when the script is executed directly
    main()
```

edit `setup.py` add this code 

```
 entry_points={
        'console_scripts': [
            "py_node = my_py_pkg.my_first_node:main"
        ],
```

Build and run your python node: 

```
cd ~/ros2_ws/
source ~/.bashrc
colcon bulid --packages-select my_cpy_package
ros2 run my_cpy_package py_node
```

**C++ :**

Use the following sequence of commands to create a new C++ node. Let's name our node first_cpp_node.

```
cd ~/ros2_ws/src/my_cpp_package/src
touch first_cpp_node.cpp
cd ~/ros2_ws/
colcon build --packages-select my_cpp_package
source ~/.bashrc
```

Add template code for the C++ node `first_cpp_node.cpp:`

```
#include "rclcpp/rclcpp.hpp"

// Define a custom node class inheriting from rclcpp::Node.
class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    // Constructor for MyCustomNode.
    MyCustomNode() : Node("node_name") // MODIFY NAME
    {
        // Print "Hello World" using the logger upon construction.
        RCLCPP_INFO(this->get_logger(), "Hello World");
    }

private:
};

// Main function
int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create an instance of MyCustomNode
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME

    // Spin the node, i.e., process any incoming messages and callbacks.
    rclcpp::spin(node);

    // Shutdown ROS2
    rclcpp::shutdown();

    return 0;
}
```

Add rclcpp path by foloing this steps 

Open Visual Studio Code.

Press `Ctrl+Shift+P`

Type C/C++: Edit Configurations (JSON) and press Enter.

Edit `c_cpp_properties.json` add this code 

```
"includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/rclcpp/**",
            ],
```

Edit `CMakeLists.txt`

Add the following lines to CMakeLists.txt in your package to ensure that the new C++ node is built and executable:

```
add_executable(cpp_node src/first_cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)
```

Build and run your C++ node:

Use the following commands to build and run your C++ node:

```
cd ~/ros2_ws/
colcon build --packages-select my_cpp_package
ros2 run my_cpp_package first_cpp_node
```
