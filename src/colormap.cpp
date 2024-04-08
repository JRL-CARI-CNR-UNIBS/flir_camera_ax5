#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <chrono>

class ColorMapNode : public rclcpp::Node
{
public:
	ColorMapNode() 
	: rclcpp::Node("colormap_node")
	{
		pub_ = this->create_publisher<sensor_msgs::msg::Image>("ax5_camera_node/image_colormap", 1);
		sub_ = this->create_subscription<sensor_msgs::msg::Image>("ax5_camera_node/image_raw", 1, 
																  [this](const sensor_msgs::msg::Image& msg) -> void {
																			std::chrono::time_point tic = std::chrono::steady_clock::now();
																			this->_cvb = cv_bridge::toCvCopy(msg, "bgr8");
																			cv::applyColorMap(this->_cvb->image, this->_cvb->image, cv::COLORMAP_MAGMA);
																			this->_cvb->toImageMsg(to_send_);
																			this->pub_->publish(to_send_);
																			RCLCPP_DEBUG_STREAM(this->get_logger(), (std::chrono::steady_clock::now() - tic).count());
																  }
		);
	}
private:
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
	cv_bridge::CvImagePtr _cvb;
	sensor_msgs::msg::Image to_send_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ColorMapNode>();
	rclcpp::spin(node);
	
	return 0;
}
