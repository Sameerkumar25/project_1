#pragma once
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
namespace cpp_pubsub
{
    class MinimalSubscriber: public rclcpp::Node
    {
        public:

        MinimalSubscriber();
        bool has_data_been_received();
        std_msgs::msg::String get_received_msg();
        double get_frequency();
        
        private:

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr topic_sub_;
        std_msgs::msg::String received_msg_;
        std::shared_ptr<std::chrono::time_point<std::chrono::system_clock>> last_message_timestamp_;
        double frequency_;
        void topic_callback(const std_msgs::msg::String & msg);
        
        
        
        
        
    };
    
}
