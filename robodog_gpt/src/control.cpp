#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "robodog_gpt/gpt_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "ros2_unitree_legged_msgs/msg/bms_cmd.hpp"

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "convert.h"
#include <functional>
#include "geometry_msgs/msg/twist.hpp"
#include "robodog_gpt/srv/comms.hpp"
#include "robodog_gpt/srv/audio.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr int MODE = 2;
constexpr int GAIT = 2;
constexpr double FOOT_HEIGHT = 0.1;
constexpr float FORWARD_VEL = 0.4f;
constexpr float BACKWARD_VEL = -0.4f;
constexpr float RIGHT_STRAFE_VEL = 0.4f;
constexpr float LEFT_STRAFE_VEL = -0.4f;
constexpr float ROTATE_RIGHT = 2.0f;
constexpr float ROTATE_LEFT = -2.0f;

class ControlNode : public rclcpp::Node {
private:
    rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr gpt_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr command_pub_;

    rclcpp::Time last_twist_time;
    geometry_msgs::msg::Twist latest_twist;
    robodog_gpt::srv::Comms::Response::SharedPtr gpt_response;
    ros2_unitree_legged_msgs::msg::HighCmd high_cmd;

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void comms_callback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg);

public:
    ControlNode() : Node("control_node") {
        command_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 10);

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/control", 10, std::bind(&ControlNode::twist_callback, this, std::placeholders::_1));
        gpt_sub_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>(
            "/comms", 10, std::bind(&ControlNode::comms_callback, this, std::placeholders::_1));

        // timer = this->create_wall_timer(20ms, std::bind(&Control::control_loop, this));
        last_twist_time = this->now();
        high_cmd.head[0] = 0xFE;
        high_cmd.head[1] = 0xEF;
        high_cmd.level_flag = HIGHLEVEL;
        high_cmd.mode = 0;
        high_cmd.gait_type = 0;
        high_cmd.speed_level = 0;
        high_cmd.foot_raise_height = 0;
        high_cmd.body_height = 0;
        high_cmd.euler[0] = 0;
        high_cmd.euler[1] = 0;
        high_cmd.euler[2] = 0;
        high_cmd.velocity[0] = 0.0f;
        high_cmd.velocity[1] = 0.0f;
        high_cmd.yaw_speed = 0.0f;
        high_cmd.reserve = 0;

        RCLCPP_INFO(this->get_logger(), "Control node initialized.");
    }
};


void ControlNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_twist = *msg;
    last_twist_time = this->now();
    ros2_unitree_legged_msgs::msg::HighCmd high_cmd;
    
    high_cmd.mode = MODE;
    high_cmd.gait_type = GAIT;
    high_cmd.speed_level = 0;
    high_cmd.foot_raise_height = FOOT_HEIGHT;

    if ((this->now() - last_twist_time).seconds() > 0.5) {
        latest_twist = geometry_msgs::msg::Twist();
    }

    if (latest_twist.linear.x > 0) {
        high_cmd.velocity[0] = FORWARD_VEL;
    } else if (latest_twist.linear.x < 0) {
        high_cmd.velocity[0] = BACKWARD_VEL;
    } else {
        high_cmd.velocity[0] = 0.0f;
    }

    if (latest_twist.linear.y > 0) {
        high_cmd.velocity[1] = LEFT_STRAFE_VEL;
    } else if (latest_twist.linear.y < 0) {
        high_cmd.velocity[1] = RIGHT_STRAFE_VEL;
    } else {
        high_cmd.velocity[1] = 0.0f;
    }

    if (latest_twist.angular.z > 0) {
        high_cmd.yaw_speed = ROTATE_LEFT;
    } else if (latest_twist.angular.z < 0) {
        high_cmd.yaw_speed = ROTATE_RIGHT;
    } else {
        high_cmd.yaw_speed = 0.0f;
    }

    command_pub_ -> publish(high_cmd);
}


void ControlNode::comms_callback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg) {
    high_cmd.mode = msg->mode;
    high_cmd.gait_type = msg->gait_type;
    high_cmd.speed_level = msg->speed_level;
    high_cmd.foot_raise_height = msg->foot_raise_height;
    high_cmd.body_height = msg->body_height;
    high_cmd.position[0] = msg->position[0];
    high_cmd.position[1] = msg->position[1];
    high_cmd.euler[0] = msg->euler[0];
    high_cmd.euler[1] = msg->euler[1];
    high_cmd.euler[2] = msg->euler[2];
    high_cmd.velocity[0] = msg->velocity[0];
    high_cmd.velocity[1] = msg->velocity[1];
    high_cmd.yaw_speed = msg->yaw_speed;
    high_cmd.reserve = msg->reserve;

    command_pub_->publish(high_cmd);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
