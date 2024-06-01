#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpp_pubsub/subscriber_member_function.hpp"


using std::placeholders::_1;
using namespace std;
namespace cpp_pubsub
{
MinimalSubscriber::MinimalSubscriber():
Node("minimal_subscriber")
	{
 		topic_sub_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
 	}
 	
void MinimalSubscriber::topic_callback(const std_msgs::msg::String & msg)
 
{
		if(!last_message_timestamp_)
  	{
			last_message_timestamp_ = std::make_shared<std::chrono::time_point<std::chrono::system_clock>>();
			*last_message_timestamp_ = std::chrono::system_clock::now();
  	}
  		else{
			auto now = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = now - *last_message_timestamp_;
			double elapsec_seconds_double = elapsed_seconds.count();
			frequency_ = 1/elapsec_seconds_double;
			*last_message_timestamp_ = now;
  	}
	received_msg_ = msg;
 }
 
 bool MinimalSubscriber::has_data_been_received()
 {
    return (!received_msg_.data.empty());
 }
 
 std_msgs::msg::String MinimalSubscriber:: get_received_msg()
 {
    return received_msg_;
 }
 
 double MinimalSubscriber::get_frequency()
 {
    std::cout << frequency_;
    return frequency_;
 }
 
}
