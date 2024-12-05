// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

// Cv bridge to convert from ROS image to OpenCV image
#include <cv_bridge/cv_bridge.h>

// Standard library includes
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

// For timing the BRAM transfer
#include <time.h>
#include <chrono>

// BRAM class
#include "bram_uio.h"

// For timekeeping;
std::chrono::_V2::system_clock::time_point t1;

void start_timer()
{
	t1 = std::chrono::high_resolution_clock::now();
	// std::cout << "Start timer" << std::endl;
}
double stop_timer()
{
	auto t2 = std::chrono::high_resolution_clock::now();
	auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
	std::chrono::duration<double, std::milli> ms_double = t2 - t1;
	// std::cout << "Duration: " << ms_double.count() << "ms [" << (float)LENGTH / 1000000. << "MB]" << std::endl;
	return ms_double.count();
}

#define IMG_WIDTH 10
#define IMG_HEIGHT 10
#define BRAM_UIO_0 0

// Class to handle the image subscription
class BramNode : public rclcpp::Node
{
	public:
		BramNode() : Node("bram_node") {
			RCLCPP_INFO(this->get_logger(), "Initializing BRAM node");

			RCLCPP_INFO(this->get_logger(), "Starting subscription to downscaled image");

			img_subscription = this->create_subscription<sensor_msgs::msg::Image>(
					"/image_downscale",
					10,
					std::bind(&BramNode::onImageMsg, this, std::placeholders::_1)
			);

			start_subscription = this->create_subscription<std_msgs::msg::String>(
					"/start",
					10,
					std::bind(&BramNode::onStartMsg, this, std::placeholders::_1)
			);

			number_publish = this->create_publisher<std_msgs::msg::Int32>(
					"/draw_number",
					10
			);

			RCLCPP_INFO(this->get_logger(), "BRAM node initialized");
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_subscription;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr number_publish;
		cv::Mat img;

		void onStartMsg(const std_msgs::msg::String::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received string message: %s", msg->data.c_str());

			BRAM bram(BRAM_UIO_0, 130*sizeof(uint32_t));
			
			RCLCPP_INFO(this->get_logger(), "BRAM initialized");

			if (msg->data == "s")
			{
				RCLCPP_INFO(this->get_logger(), "Starting BRAM transfer");

				// Iterate over each pixel of the image and write it to BRAM
				for (int row = 0; row < IMG_HEIGHT; row++)
				{
					for (int col = 0; col < IMG_WIDTH; col++)
					{
						// Write to bram
						bram[row*IMG_WIDTH + col] = (uint32_t)img.at<uint8_t>(row, col);
						std::cout << (uint32_t)img.at<uint8_t>(row, col) << " ";
					}
					std::cout << std::endl;
				}

				// Wait for the Neural Network to predict number and write it to BRAM
				sleep(0.1);

				// Retrive the number from BRAM and publish to topic
				std_msgs::msg::Int32 number_msg;
				number_msg.data = bram[128];
				number_publish->publish(number_msg);
			}
		}

		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received image!");

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			img = cv_ptr->image;			
		}

};

int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<BramNode>());

	rclcpp::shutdown();
	return 0;
}
