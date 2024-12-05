// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

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

// Class to handle the image subscription
class ImageSubscriber : public rclcpp::Node
{
	public:
		ImageSubscriber() : Node("image_subscriber") {
			RCLCPP_INFO(this->get_logger(), "Initializing ImageSubscriber node");

			RCLCPP_INFO(this->get_logger(), "Starting camera subscription");

			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
					"/image_raw",
					10,
					std::bind(&ImageSubscriber::onImageMsg, this, std::placeholders::_1)
			);

            bw_publisher = this->create_publisher<sensor_msgs::msg::Image>("/image_bw", 10);
            dilate_publisher = this->create_publisher<sensor_msgs::msg::Image>("/image_dilate", 10);
			camera_publisher = this->create_publisher<sensor_msgs::msg::Image>("/image_downscale", 10);
			threshold_publisher = this->create_publisher<sensor_msgs::msg::Image>("/image_threshold", 10);
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bw_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dilate_publisher;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr threshold_publisher;

		void onImageMsg(const sensor_msgs::msg::Image::SharedPtr msg) {
			RCLCPP_INFO(this->get_logger(), "Received image!");

			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
			cv::Mat cam_img = cv_ptr->image;

            // Get the current dimensions of the image
            int height = cam_img.rows;
            int width = cam_img.cols;

            // Determine the size of the square (the smaller of height or width)
            int newSize = std::min(height, width);

            // Calculate the cropping region
            int xOffset = (width - newSize) / 2;
            int yOffset = (height - newSize) / 2;

            // Crop the image to make it square
            cv::Rect cropRegion(xOffset, yOffset, newSize, newSize);
            cv::Mat croppedImage = cam_img(cropRegion);

			cv::Mat gray_img(croppedImage.rows, croppedImage.cols, CV_8UC1);
			cv::cvtColor(croppedImage, gray_img, cv::COLOR_YUV2GRAY_YUYV);

             // Define a threshold for black pixels (close to 0 intensity)
            int lowerThreshold = 0;  // Adjust if necessary
            int upperThreshold = 120; // This range can be adjusted based on your image

            // Threshold the image to get a binary mask of black pixels
            cv::Mat blackPixelsMask;
            cv::inRange(gray_img, lowerThreshold, upperThreshold, blackPixelsMask);

            // Convert the binary mask to a ROS message
            sensor_msgs::msg::Image::SharedPtr bw_message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", blackPixelsMask).toImageMsg();
            bw_publisher->publish(*bw_message.get());

            cv::Mat dilated;
            cv::Mat kernel(10, 10, CV_8U, cv::Scalar(1));
            cv::dilate(blackPixelsMask, dilated, kernel, cv::Point(-1, -1), 2);

            // Convert the dilated image to a ROS message
            sensor_msgs::msg::Image::SharedPtr dilate_message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", dilated).toImageMsg();
            dilate_publisher->publish(*dilate_message.get());

            cv::Mat downscale_img;
            cv::resize(dilated, downscale_img, cv::Size(10, 10), 0, 0, cv::INTER_LINEAR);

			// Convert the image to a ROS message
            sensor_msgs::msg::Image::SharedPtr downscale_message = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", downscale_img).toImageMsg();
            camera_publisher->publish(*downscale_message.get());
		}

};

int main(int argc, char *argv[])
{
	setvbuf(stdout,NULL,_IONBF,BUFSIZ);

	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<ImageSubscriber>());

	rclcpp::shutdown();
	return 0;
}
