# Services client

This guide walks you through Create Services client. Follow the steps below:

## Step 1: Create Package 

Ensure that you have created your package. If not, follow these [steps](../Create_Package/README.md)


## Step 2: Create client


**python :**

Use This sequence of commands to create client. Let's call our Node add_two_ints_client.py


```
cd ~/ros2_ws/src/my_py_package/my_py_package
touch add_two_ints_client.py
chmod +x add_two_ints_client.py
```

add this template code for the python Node add_two_ints_client.py

```
#!/usr/bin/env python3
# This line specifies the interpreter to use for executing this script.

import rclpy
# Importing the rclpy module which provides ROS 2 client library for Python.
from rclpy.node import Node
# Importing the Node class from the rclpy.node module.
from functools import partial
# Importing the partial function from the functools module.

from example_interfaces.srv import AddTwoInts
# Importing the AddTwoInts service message from the example_interfaces package.

class AddTwoIntsClientNode(Node):
    # Defining a class for the node that will request the AddTwoInts service.

    def __init__(self):
        # Constructor method for initializing the node.
        super().__init__("add_two_ints_client")
        # Calling the constructor of the superclass with the node name.
        self.call_add_two_ints_server(6, 7)
        # Calling the add_two_ints_server method with arguments 6 and 7.
        self.call_add_two_ints_server(3, 1)
        # Calling the add_two_ints_server method with arguments 3 and 1.
        self.call_add_two_ints_server(5, 2)
        # Calling the add_two_ints_server method with arguments 5 and 2.

    def call_add_two_ints_server(self, a, b):
        # Method to call the AddTwoInts service server with given integers a and b.
        client = self.create_client(AddTwoInts, "add_two_ints")
        # Creating a client to call the AddTwoInts service.
        while not client.wait_for_service(1.0):
            # Waiting until the service is available.
            self.get_logger().warn("Waiting for Server Add Two Ints...")
            # Logging a warning message while waiting for the service.

        request = AddTwoInts.Request()
        # Creating a request object for the AddTwoInts service.
        request.a = a
        # Setting the value of 'a' in the request.
        request.b = b
        # Setting the value of 'b' in the request.

        future = client.call_async(request)
        # Calling the service asynchronously and storing the future object.
        future.add_done_callback(
            partial(self.callback_call_add_two_ints, a=a, b=b))
        # Adding a callback function to handle the result of the service call.

    def callback_call_add_two_ints(self, future, a, b):
        # Callback method to handle the result of the service call.
        try:
            response = future.result()
            # Getting the result of the service call.
            self.get_logger().info(str(a) + " + " +
                                   str(b) + " = " + str(response.sum))
            # Logging the request and the result.
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
            # Logging an error message if the service call fails.

def main(args=None):
    # Main function to initialize the node and start spinning.
    rclpy.init(args=args)
    # Initializing the ROS 2 client library.
    node = AddTwoIntsClientNode()
    # Creating an instance of the AddTwoIntsClientNode class.
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
            "add_two_ints_client = my_py_pkg.add_two_ints_client:main",
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
ros2 run my_py_package add_two_ints_client
```

In another terminal, execute the following command:

```
ros2 run my_py_package add_two_ints_server
```


**C++ :**

Use This sequence of commands to create client. Let's call our Node add_two_ints_client.cpp

```
cd ~/ros2_ws/src/my_cpp_pkg/src/
touch add_two_ints_client.cpp
```

add this template code for the python Node add_two_ints_client.cpp

```
#include "rclcpp/rclcpp.hpp"
// Including the ROS 2 C++ client library header file.

#include "example_interfaces/srv/add_two_ints.hpp"
// Including the service message header file.

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        // Constructor for the client node, creating threads to call the service with different arguments.
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 4, 5)));
    }

    void callAddTwoIntsService(int a, int b)
    {
        // Method to call the AddTwoInts service with given integers a and b.
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        // Creating a client to call the AddTwoInts service.
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            // Logging a warning message while waiting for the service to be available.
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        // Creating a request object for the AddTwoInts service.
        request->a = a;
        // Setting the value of 'a' in the request.
        request->b = b;
        // Setting the value of 'b' in the request.

        auto future = client->async_send_request(request);
        // Sending the request asynchronously and storing the future object.

        try
        {
            auto response = future.get();
            // Getting the response from the future object.
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, (int)response->sum);
            // Logging the request and the response.
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
            // Logging an error message if the service call fails.
        }
    }

private:
    std::vector<std::thread> threads_;
    // Declaration of a vector to store threads for calling the service.
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Initializing the ROS 2 client library.
    auto node = std::make_shared<AddTwoIntsClientNode>();
    // Creating an instance of the AddTwoIntsClientNode class.
    rclcpp::spin(node);
    // Spinning the node to handle incoming requests.
    rclcpp::shutdown();
    // Shutting down the ROS 2 client library.
    return 0;
}
```

edit `CMakeLists.txt` add this code 

```
add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

install(TARGETS
  add_two_ints_client
  DESTINATION lib/${PROJECT_NAME}
)
```

Build and run your c++ node: 

```
cd ~/ros2_ws/
source ~/.bashrc
colcon bulid --packages-select my_cpp_package
ros2 run my_cpp_package add_two_ints_client
```

In another terminal, execute the following command:

```
ros2 run my_cpp_package add_two_ints_server
```