#define BOOST_BIND_GLOBAL_PLACEHOLDERS
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
#include "robodog_gpt/msg/comms.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;
using std::placeholders::_1;

class ControlNode : public rclcpp::Node {
    private:
        rclcpp::Subscription<robodog_gpt::msg::Comms>::SharedPtr gpt_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr command_pub_;
        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Time last_twist_time;
        geometry_msgs::msg::Twist latest_twist_;
        ros2_unitree_legged_msgs::msg::HighCmd high_cmd_;
        bool gpt_response_ = false;
        rclcpp::Time gpt_command_start_time_;
        rclcpp::Duration gpt_command_duration_ = rclcpp::Duration::from_seconds(0.0);

        int mode;
        int gait;
        double foot_height;
        double forward_vel;
        double backward_vel;
        double right_strafe_vel;
        double left_strafe_vel;
        double rotate_right;
        double rotate_left;

        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void comms_callback(const robodog_gpt::msg::Comms::SharedPtr msg);
        void control_loop();
        void reset_commands();

    public:
        ControlNode() : Node("control_node") {
            command_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 10);
            twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/control_twist", 10, std::bind(&ControlNode::twist_callback, this, std::placeholders::_1));
            gpt_sub_ = this->create_subscription<robodog_gpt::msg::Comms>("/control_gpt", 10, std::bind(&ControlNode::comms_callback, this, std::placeholders::_1));

            timer = this->create_wall_timer(20ms, std::bind(&ControlNode::control_loop, this));

            this->declare_parameter<int>("mode", 2);
            this->declare_parameter<int>("gait", 2);
            this->declare_parameter<double>("foot_height", 0.1);
            this->declare_parameter<double>("forward_vel", 0.4);
            this->declare_parameter<double>("backward_vel", -0.4);
            this->declare_parameter<double>("right_strafe_vel", 0.4);
            this->declare_parameter<double>("left_strafe_vel", -0.4);
            this->declare_parameter<double>("rotate_right", 2.0);
            this->declare_parameter<double>("rotate_left", -2.0);

            mode = this->get_parameter("mode").as_int();
            gait = this->get_parameter("gait").as_int();
            foot_height = this->get_parameter("foot_height").as_double();
            forward_vel = this->get_parameter("forward_vel").as_double();
            backward_vel = this->get_parameter("backward_vel").as_double();
            right_strafe_vel = this->get_parameter("right_strafe_vel").as_double();
            left_strafe_vel = this->get_parameter("left_strafe_vel").as_double();
            rotate_right = this->get_parameter("rotate_right").as_double();
            rotate_left = this->get_parameter("rotate_left").as_double();

            last_twist_time = this->now();
            reset_commands();
            
            RCLCPP_INFO(this->get_logger(), "Control node initialized.");
        }
};


void ControlNode::reset_commands(){
    high_cmd_.head[0] = 0xFE;
    high_cmd_.head[1] = 0xEF;
    high_cmd_.level_flag = HIGHLEVEL;
    high_cmd_.mode = 0;
    high_cmd_.gait_type = 0;
    high_cmd_.speed_level = 0;
    high_cmd_.foot_raise_height = 0;
    high_cmd_.body_height = 0;
    high_cmd_.euler[0] = 0;
    high_cmd_.euler[1] = 0;
    high_cmd_.euler[2] = 0;
    high_cmd_.velocity[0] = 0.0f;
    high_cmd_.velocity[1] = 0.0f;
    high_cmd_.yaw_speed = 0.0f;
    high_cmd_.reserve = 0;
}

void ControlNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    latest_twist_ = *msg;
    last_twist_time = this->now();
    gpt_response_ = false;
}

void ControlNode::comms_callback(const robodog_gpt::msg::Comms::SharedPtr msg) {
    gpt_response_ = true;
    gpt_command_start_time_ = this->now();
    high_cmd_.mode = msg->mode;
    high_cmd_.gait_type = msg->gait_type;
    high_cmd_.speed_level = msg->speed_level;
    high_cmd_.foot_raise_height = msg->foot_raise_height;
    high_cmd_.body_height = msg->body_height;
    high_cmd_.position[0] = msg->position[0];
    high_cmd_.position[1] = msg->position[1];
    high_cmd_.euler[0] = msg->euler[0];
    high_cmd_.euler[1] = msg->euler[1];
    high_cmd_.euler[2] = msg->euler[2];
    high_cmd_.velocity[0] = msg->velocity[0];
    high_cmd_.velocity[1] = msg->velocity[1];
    high_cmd_.yaw_speed = msg->yaw_speed;
    high_cmd_.reserve = msg->reserve;

    gpt_command_duration_ = rclcpp::Duration::from_seconds(msg->max_motiontime);
}


void ControlNode::control_loop() {
    if (gpt_response_) {
        rclcpp::Time now = this->now();
        if (now - gpt_command_start_time_ < gpt_command_duration_) {
            command_pub_->publish(high_cmd_);
            return;
        } else {
            gpt_response_ = false;
            reset_commands();
        }
    } else {
        high_cmd_.mode = mode;
        high_cmd_.gait_type = gait;
        high_cmd_.speed_level = 0;
        high_cmd_.foot_raise_height = foot_height;

        if ((this->now() - last_twist_time).seconds() > 0.5) {
            latest_twist_ = geometry_msgs::msg::Twist();
        } else {
            if (latest_twist_.linear.x > 0) high_cmd_.velocity[0] = forward_vel;
            else if (latest_twist_.linear.x < 0) high_cmd_.velocity[0] = backward_vel;
            else high_cmd_.velocity[0] = 0.0;

            if (latest_twist_.linear.y > 0) high_cmd_.velocity[1] = left_strafe_vel;
            else if (latest_twist_.linear.y < 0) high_cmd_.velocity[1] = right_strafe_vel;
            else high_cmd_.velocity[1] = 0.0;

            if (latest_twist_.angular.z > 0) high_cmd_.yaw_speed = rotate_left;
            else if (latest_twist_.angular.z < 0) high_cmd_.yaw_speed = rotate_right;
            else high_cmd_.yaw_speed = 0.0;
        }
        
        command_pub_->publish(high_cmd_);
    }       
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
