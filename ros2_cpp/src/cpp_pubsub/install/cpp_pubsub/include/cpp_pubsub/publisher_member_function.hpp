#pragma once
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace cpp_pubsub
{
    class MinimalPublisher: public rclcpp::Node
    {
        public:
        
        MinimalPublisher();
        void timer_callback();
        std::string v_data;
	std::string value;
        private:
        
        rclcpp::TimerBase::SharedPtr timer_;
    	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr topic_pub_;
    	std::size_t count_;
        std_msgs::msg::String message;
        std::string data;


    };
}


