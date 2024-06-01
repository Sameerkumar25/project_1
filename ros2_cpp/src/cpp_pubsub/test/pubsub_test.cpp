#include <string>
#include "cpp_pubsub/publisher_member_function.hpp"
#include "cpp_pubsub/subscriber_member_function.hpp"
#include "cpp_pubsub/single_thread_spinner.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

TEST(PubSubCommunication,check_msg_type)
{
    rclcpp::init(0,nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    auto spinner = cpp_pubsub::SingleThreadSpinner();
    auto pub_node = std::make_shared<cpp_pubsub::MinimalPublisher>();
    auto sub_node = std::make_shared<cpp_pubsub::MinimalSubscriber>();
    
    EXPECT_FALSE(sub_node->has_data_been_received());
    

    spinner.add_node(pub_node->get_node_base_interface());
    spinner.add_node(sub_node->get_node_base_interface());

    spinner.spin_some_all_nodes();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    EXPECT_TRUE(sub_node->has_data_been_received());
    
    EXPECT_EQ(typeid(sub_node->get_received_msg().data.c_str()).name(),typeid(pub_node->v_data.c_str()).name());
    
    
    rclcpp::shutdown();
    
}


TEST(PubSubCommunication,check_msg_range)
{
    rclcpp::init(0,nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    auto spinner = cpp_pubsub::SingleThreadSpinner();
    auto pub_node = std::make_shared<cpp_pubsub::MinimalPublisher>();
    auto sub_node = std::make_shared<cpp_pubsub::MinimalSubscriber>();
    
    EXPECT_FALSE(sub_node->has_data_been_received());
    

    spinner.add_node(pub_node->get_node_base_interface());
    spinner.add_node(sub_node->get_node_base_interface());

    spinner.spin_some_all_nodes();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    EXPECT_TRUE(sub_node->has_data_been_received());
    
    EXPECT_GE(pub_node->v_data.length(),3);
    EXPECT_LT(pub_node->v_data.length(),50);
    
    
    if ((pub_node->v_data.length()<3)||((pub_node->v_data.length()>50)))
    {
    	//EXPECT_GT(pub_node->v_data.length(),3);
    	//EXPECT_LT(pub_node->v_data.length(),50);
    	
    	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message Range is Invalid");
    }
    /*
    else{
    
    	EXPECT_GE(pub_node->v_data.length(),3);
    	EXPECT_LT(pub_node->v_data.length(),50);
    	
    	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Message Range is valid");
    }
    */
    
    rclcpp::shutdown();
    
}
TEST(PubSubCommunication,pub_sub_communication)
{
    rclcpp::init(0,nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    auto spinner = cpp_pubsub::SingleThreadSpinner();
    auto pub_node = std::make_shared<cpp_pubsub::MinimalPublisher>();
    auto sub_node = std::make_shared<cpp_pubsub::MinimalSubscriber>();
    
    EXPECT_FALSE(sub_node->has_data_been_received());
    

    spinner.add_node(pub_node->get_node_base_interface());
    spinner.add_node(sub_node->get_node_base_interface());

    spinner.spin_some_all_nodes();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    EXPECT_TRUE(sub_node->has_data_been_received());
    
    rclcpp::shutdown();
    
}

TEST(PubSubCommunication,check_received_msg)
{
    rclcpp::init(0,nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    auto spinner = cpp_pubsub::SingleThreadSpinner();
    auto pub_node = std::make_shared<cpp_pubsub::MinimalPublisher>();
    auto sub_node = std::make_shared<cpp_pubsub::MinimalSubscriber>();

    
    EXPECT_FALSE(sub_node->has_data_been_received());


    spinner.add_node(pub_node->get_node_base_interface());
    spinner.add_node(sub_node->get_node_base_interface());

    spinner.spin_some_all_nodes();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    EXPECT_TRUE(sub_node->has_data_been_received());
    
    EXPECT_STREQ(sub_node->get_received_msg().data.c_str(),pub_node->v_data.c_str());
    
    rclcpp::shutdown();
}
TEST(PubSubCommunication,check_frequency)
{
    rclcpp::init(0,nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    
    auto spinner = cpp_pubsub::SingleThreadSpinner();
    auto pub_node = std::make_shared<cpp_pubsub::MinimalPublisher>();
    auto sub_node = std::make_shared<cpp_pubsub::MinimalSubscriber>();

    EXPECT_FALSE(sub_node->has_data_been_received());
    
    spinner.add_node(pub_node->get_node_base_interface());
    spinner.add_node(sub_node->get_node_base_interface());

    spinner.spin_some_all_nodes();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    EXPECT_TRUE(sub_node->has_data_been_received());
    
    EXPECT_EQ(round(sub_node->get_frequency()), 1);
    
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}

