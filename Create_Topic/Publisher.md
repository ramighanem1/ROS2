# Topic Publisher

This guide walks you through Create Topic Publisher. Follow the steps below:

## Step 1: Create Package 

Ensure that you have created your package. If not, follow these [steps](../Create_Package/README.md)

## Step 2: Create Publisher

**python :**

Use This sequence of commands to create Publisher. Let's call our Node robot_news_station.py

```
cd ~/ros2_ws/src/my_py_package/my_py_package
touch robot_news_station.py
chmod +x robot_news_station.py
```

add this template code for the python Node robot_news_station.py

```
import rclpy  # Importing the ROS2 Python library for communication with ROS 2 nodes.
from rclpy.node import Node  # Importing the Node class for creating ROS2 nodes.

from example_interfaces.msg import String  # Importing the String message type from example_interfaces.


class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")  # Initializing the node with the name "robot_news_station".

        self.robot_name_ = "C3PO"  # Setting the robot name.
        self.publisher_ = self.create_publisher(String, "robot_news", 10)  # Creating a publisher that publishes messages of type String on the topic "robot_news".
        self.timer_ = self.create_timer(0.5, self.publish_news)  # Creating a timer that triggers the publish_news method every 0.5 seconds.
        self.get_logger().info("Robot News Station has been started")  # Logging an info message indicating that the node has been started.

    def publish_news(self):
        msg = String()  # Creating a String message object.
        msg.data = "Hi, this is " + \
            str(self.robot_name_) + " from the robot news station."  # Setting the data field of the message with a formatted string.
        self.publisher_.publish(msg)  # Publishing the message.


def main(args=None):
    rclpy.init(args=args)  # Initializing the ROS 2 Python client library.
    node = RobotNewsStationNode()  # Creating an instance of the RobotNewsStationNode class.
    rclpy.spin(node)  # Spinning the node to handle callbacks.
    rclpy.shutdown()  # Shutting down the ROS 2 Python client library when the node is done.


if __name__ == "__main__":
    main()  # Entry point of the program, calling the main function.

```

edit `setup.py` add this code 

```
 entry_points={
        'console_scripts': [
            "robot_news_station = my_py_pkg.robot_news_station:main",
        ],
```

edit `package.xml` add this code 

```
   <depend>example_interfaces</depend>
```

Build and run your python node: 

```
cd ~/ros2_ws/
source ~/.bashrc
colcon bulid --packages-select my_py_package --symlink-install
ros2 run my_py_package py_node
```

In another terminal, execute the following command:

```
ros2 topic echo /robot_news 
```

**C++ :**

Use This sequence of commands to create Publisher. Let's call our Node robot_news_station.cpp

```
cd ~/ros2_ws/src/my_cpp_pkg/src/
touch robot_news_station.cpp
```

add this template code for the python Node robot_news_station.cpp

```
#include "rclcpp/rclcpp.hpp"  // Include the ROS C++ library
#include "example_interfaces/msg/string.hpp"  // Include the String message type

// Class definition for the RobotNewsStationNode, inheriting from rclcpp::Node
class RobotNewsStationNode : public rclcpp::Node
{
public:
    // Constructor for the node
    RobotNewsStationNode() : Node("robot_news_station"), robot_name_("R2D2")
    {
        // Creating a publisher for the "robot_news" topic
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        // Creating a timer to publish news periodically
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&RobotNewsStationNode::publishNews, this));
        // Logging a message to indicate that the Robot News Station node has been started
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started.");
    }

private:
    // Function to publish news
    void publishNews()
    {
        // Creating a String message
        auto msg = example_interfaces::msg::String();
        // Composing the message content
        msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the Robot News Station");
        // Publishing the message
        publisher_->publish(msg);
    }

    std::string robot_name_;  // Name of the robot
    // Publisher for the "robot_news" topic
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    // Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;
};

// Main function
int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    // Create an instance of the RobotNewsStationNode
    auto node = std::make_shared<RobotNewsStationNode>();
    // Spin the node
    rclcpp::spin(node);
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}

```

edit `CMakeLists.txt` add this code 

```
find_package(example_interfaces REQUIRED)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

install(TARGETS
  robot_news_station
  DESTINATION lib/${PROJECT_NAME}
)
```

edit `package.xml` add this code 

```
   <depend>example_interfaces</depend>
```

Build and run your c++ node: 

```
cd ~/ros2_ws/
source ~/.bashrc
colcon bulid --packages-select my_cpp_package
ros2 run my_cpp_package robot_news_station
```

In another terminal, execute the following command:

```
ros2 topic echo /robot_news 
```