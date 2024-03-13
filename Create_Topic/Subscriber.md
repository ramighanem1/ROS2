# Topic Subscribers

This guide walks you through Create Topic Subscriber. Follow the steps below:

## Step 1: Create Package 

Ensure that you have created your package. If not, follow these [steps](../Create_Package/README.md)

## Step 2: Create Subscriber

**python :**

Use This sequence of commands to create Subscriber. Let's call our Node smartphone.py

```
cd ~/ros2_ws/src/my_py_package/my_py_package
touch smartphone.py
chmod +x smartphone.py
```

add this template code for the python Node smartphone.py

```
#!/usr/bin/env python3
# Importing necessary libraries
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Node class for creating ROS nodes

from example_interfaces.msg import String  # Importing String message type


# Defining a class for SmartphoneNode
class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")  # Initialize the node with the name "smartphone"
        # Creating a subscriber to the "robot_news" topic
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10)
        # Logging a message to indicate that the smartphone node has been started
        self.get_logger().info("Smartphone has been started.")

    # Callback function for receiving messages on the "robot_news" topic
    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)  # Logging the received message


# Main function to initialize ROS 2 and start the smartphone node
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2
    node = SmartphoneNode()  # Create an instance of SmartphoneNode
    rclpy.spin(node)  # Spin the node to keep it running
    rclpy.shutdown()  # Shutdown ROS 2 when done


# Entry point of the script
if __name__ == "__main__":
    main()  # Call the main function to start the program


```

edit `setup.py` add this code 

```
 entry_points={
        'console_scripts': [
            "smartphone = my_py_pkg.smartphone:main",
        ],
```


Build and run your python node: 

```
cd ~/ros2_ws/
source ~/.bashrc
colcon bulid --packages-select my_py_package --symlink-install
ros2 run my_py_package smartphone
```

In another terminal, execute the following command:

```
ros2 run my_py_package robot_news_station
```

**C++ :**

Use This sequence of commands to create Subscriber. Let's call our Node smartphone.cpp

```
cd ~/ros2_ws/src/my_cpp_pkg/src/
touch smartphone.cpp
```

add this template code for the python Node smartphone.cpp

```
#include "rclcpp/rclcpp.hpp"  // Include the ROS C++ library
#include "example_interfaces/msg/string.hpp"  // Include the String message type

// Class definition for the SmartphoneNode, inheriting from rclcpp::Node
class SmartphoneNode : public rclcpp::Node
{
public:
    // Constructor for the node
    SmartphoneNode() : Node("smartphone")
    {
        // Creating a subscription for the "robot_news" topic
        subscriber_ = this->create_subscription<example_interfaces::msg::String>(
            "robot_news", 10,
            std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));
        // Logging a message to indicate that the Smartphone node has been started
        RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
    }

private:
    // Callback function for receiving messages on the "robot_news" topic
    void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        // Logging the received message
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    // Subscriber for the "robot_news" topic
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

// Main function
int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    // Create an instance of the SmartphoneNode
    auto node = std::make_shared<SmartphoneNode>();
    // Spin the node
    rclcpp::spin(node);
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
```

edit `CMakeLists.txt` add this code 

```
add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

install(TARGETS
  smartphone
  DESTINATION lib/${PROJECT_NAME}
)
```

Build and run your c++ node: 

```
cd ~/ros2_ws/
source ~/.bashrc
colcon bulid --packages-select my_cpp_package
ros2 run my_cpp_package smartphone
```

In another terminal, execute the following command:

```
ros2 run my_cpp_package robot_news_station
```

