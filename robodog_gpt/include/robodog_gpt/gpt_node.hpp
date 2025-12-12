#ifndef ROBODOG_GPT_GPT_NODE_HPP
#define ROBODOG_GPT_GPT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ros2_unitree_legged_msgs/msg/high_cmd.hpp>
#include <string>

class GPTNode : public rclcpp::Node {
public:
    GPTNode();

private:
    void gpt_req(const std_msgs::msg::String::SharedPtr msg);
    void send_cmd(const std::string &json);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gpt_sub_;
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr gpt_pub_;
};

#endif