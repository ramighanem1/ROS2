# Services servier

This guide walks you through Create Services servier. Follow the steps below:

## Step 1: Create Package 

Ensure that you have created your package. If not, follow these [steps](../Create_Package/README.md)

## Step 2: Create servier

**python :**

Use This sequence of commands to create servier. Let's call our Node add_two_ints_server.py

```
cd ~/ros2_ws/src/my_py_package/my_py_package
touch radd_two_ints_server.py
chmod +x add_two_ints_server.py
```

add this template code for the python Node add_two_ints_server.py

```
#!/usr/bin/env python3
# This line specifies the interpreter to use for executing this script.

import rclpy
# Importing the rclpy module which provides ROS 2 client library for Python.

from rclpy.node import Node
# Importing the Node class from the rclpy.node module.

from example_interfaces.srv import AddTwoInts
# Importing the AddTwoInts service message from the example_interfaces package.

class AddTwoIntsServerNode(Node):
    # Defining a class for the node that will serve the AddTwoInts service.

    def __init__(self):
        # Constructor method for initializing the node.
        super().__init__("add_two_ints_server")
        # Calling the constructor of the superclass with the node name.
        self.server_ = self.create_service(
            AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        # Creating a service named "add_two_ints" with the callback method.
        self.get_logger().info("Add two ints server has been started.")
        # Logging a message indicating that the server has been started.

    def callback_add_two_ints(self, request, response):
        # Callback method to handle requests to add two integers.
        response.sum = request.a + request.b
        # Calculating the sum of the two integers and storing it in the response.
        self.get_logger().info(str(request.a) + " + " +
                               str(request.b) + " = " + str(response.sum))
        # Logging the request and the calculated sum.
        return response
        # Returning the response to the client.

def main(args=None):
    # Main function to initialize the node and start spinning.
    rclpy.init(args=args)
    # Initializing the ROS 2 client library.
    node = AddTwoIntsServerNode()
    # Creating an instance of the AddTwoIntsServerNode class.
    rclpy.spin(node)
    # Spinning the node to handle incoming requests.
    rclpy.shutdown()
    # Shutting down the ROS 2 client library.

if __name__ == "__main__":
    # Conditional to check if the script is being run directly.
    main()
    # Calling the main function if the script is being run directly.

```

edit `setup.py` add this code 

```
 entry_points={
        'console_scripts': [
            "add_two_ints_server = my_py_pkg.add_two_ints_server:main",
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
ros2 run my_py_package add_two_ints_server
```

In another terminal, execute the following command:

```
ros2 servier call /add_two_ints_server example_interfaces/srv/AddTwoInts "{a: 3 ,b: 4}"
```


**C++ :**

Use This sequence of commands to create servier. Let's call our Node add_two_ints_server.cpp

```
cd ~/ros2_ws/src/my_cpp_pkg/src/
touch add_two_ints_server.cpp
```

add this template code for the python Node add_two_ints_server.cpp

```
#include "rclcpp/rclcpp.hpp"
// Including the ROS 2 C++ client library header file.

#include "example_interfaces/srv/add_two_ints.hpp"
// Including the service message header file.

using std::placeholders::_1;
using std::placeholders::_2;
// Importing placeholders for binding arguments in callback functions.

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        // Constructor for the server node, creating a service and binding it to a callback function.
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));
        
        // Logging that the service server has been started.
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

private:
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        // Callback function to handle the service request, calculating the sum and logging it.
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
    // Declaration of the service server.
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Initializing the ROS 2 client library.
    auto node = std::make_shared<AddTwoIntsServerNode>();
    // Creating an instance of the AddTwoIntsServerNode class.
    rclcpp::spin(node);
    // Spinning the node to handle incoming requests.
    rclcpp::shutdown();
    // Shutting down the ROS 2 client library.
    return 0;
}
```

edit `CMakeLists.txt` add this code 

```
find_package(example_interfaces REQUIRED)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

install(TARGETS
  add_two_ints_server
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
ros2 run my_cpp_package add_two_ints_server
```

In another terminal, execute the following command:

```
ros2 servier call /add_two_ints_server example_interfaces/srv/AddTwoInts "{a: 3 ,b: 4}"
```

