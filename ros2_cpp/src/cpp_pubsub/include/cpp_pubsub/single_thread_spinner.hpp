#pragma once
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace cpp_pubsub
{
    class SingleThreadSpinner: private rclcpp::executors::SingleThreadedExecutor
    {
        public:
        SingleThreadSpinner();
        ~SingleThreadSpinner();
        bool add_node(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node);
        bool add_node(rclcpp::Node::SharedPtr node);
        bool remove_node(rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node);
        bool remove_node(rclcpp::Node::SharedPtr node);
        bool add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);
        bool remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);
        bool cancel_all_spin();
        bool are_all_nodes_spinning();
        bool spin_some_all_nodes();
        void spin_all_nodes_();
        
        


        private:
        //void spin_all_nodes_();
        std::mutex mutex_;
        std::shared_ptr<std::thread> spinner_thread_;
        std::atomic<bool> are_all_nodes_spinning_ = false;
        std::atomic<bool> cancel_spin_called_ = false;




    };
    
}


