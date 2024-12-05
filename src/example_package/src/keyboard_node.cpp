// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

// Standard library includes
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

class KeyboardNode : public rclcpp::Node
{
	public:
		KeyboardNode() : Node("keyboard_node") {
			RCLCPP_INFO(this->get_logger(), "Initializing Keyboard node");

			number_publish = this->create_publisher<std_msgs::msg::String>(
					"/start",
					10
			);

			RCLCPP_INFO(this->get_logger(), "Keyboard node initialized");

            while (true)
            {
                std::string input;
                std::cin >> input;

                std_msgs::msg::String start_msg;
                start_msg.data = input;
                number_publish->publish(start_msg);
            }
		}

	private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr number_publish;
};


int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<KeyboardNode>());

	rclcpp::shutdown();
	return 0;
}
