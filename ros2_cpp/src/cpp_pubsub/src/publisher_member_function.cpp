#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpp_pubsub/publisher_member_function.hpp"


using namespace std::chrono_literals;
namespace cpp_pubsub
{
MinimalPublisher::MinimalPublisher():
Node("minimal_publisher"),count_(0)
{	
	std::cout << "Enter msg ";
        std::getline(std::cin, value);
	topic_pub_ = this-> create_publisher<std_msgs::msg::String>("topic",10);
	
	
	timer_ = this-> create_wall_timer(1000ms,std::bind(&MinimalPublisher::timer_callback,this));

	
}
void MinimalPublisher::timer_callback()
{
	auto message = std_msgs::msg::String();
	v_data = value +" "+ std::to_string(count_++);
        message.data = v_data;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        topic_pub_->publish(message);
}
}
