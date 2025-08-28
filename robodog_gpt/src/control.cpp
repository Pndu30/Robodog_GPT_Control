#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "ros2_unitree_legged_msgs/msg/bms_cmd.hpp"

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "convert.h"
#include <string>
#include <functional>
#include "geometry_msgs/msg/twist.hpp"
#include "robodog_gpt/srv/comms.hpp"
#include "robodog_gpt/srv/audio.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

constexpr int MODE = 2;
constexpr int GAIT = 2;
constexpr double FOOT_HEIGHT = 0.1;
constexpr float FORWARD_VEL = 0.4f;
constexpr float BACKWARD_VEL = -0.4f;
constexpr float RIGHT_STRAFE_VEL = 0.4f;
constexpr float LEFT_STRAFE_VEL = -0.4f;
constexpr float ROTATE_RIGHT = 2.0f;
constexpr float ROTATE_LEFT = -2.0f;

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;

class Control : public rclcpp::Node{
    private:
        rclcpp::Client<robodog_gpt::srv::Comms>::SharedPtr gpt_client;
        rclcpp::Client<robodog_gpt::srv::Audio>::SharedPtr audio_client;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr msg_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr audio_sub;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Subscription<ros2_unitree_legged_msgs::msg::BmsCmd>::SharedPtr connection_checker;
        
        bool use_gpt = false;
        bool connected = false;

        rclcpp::Time gpt_command_start_time;
        rclcpp::Duration gpt_command_duration;
        
        rclcpp::Time last_twist_time;
        geometry_msgs::msg::Twist latest_twist;
        robodog_gpt::srv::Comms::Response::SharedPtr gpt_response;

        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void msg_callback(const std_msgs::msg::String::SharedPtr cmd);
        void audio_callback(const std_msgs::msg::Bool::SharedPtr state);
        void control_loop();
        void check_connection(const ros2_unitree_legged_msgs::msg::BmsCmd::SharedPtr connection);
    

    public:
        Control() : Node("control_node"){
            gpt_client = this->create_client<robodog_gpt::srv::Comms>("/comms");
            audio_client = this->create_client<robodog_gpt::srv::Audio>("/audio");
            pub = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 10);
            twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("control", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {this->twist_callback(msg);});
            msg_sub = this->create_subscription<std_msgs::msg::String>("msg", 10, [this](const std_msgs::msg::String::SharedPtr cmd) {this->msg_callback(cmd);});
            audio_sub = this->create_subscription<std_msgs::msg::Bool>("audio", 10, [this](const std_msgs::msg::Bool::SharedPtr state) {this->audio_callback(state);});
            
            timer = this->create_wall_timer(20ms, std::bind(&Control::control_loop, this));
            
            last_twist_time = this->now();
            connection_checker = this->create_subscription<ros2_unitree_legged_msgs::msg::BmsCmd>("bms_cmd", rclcpp::SensorDataQoS(), [this](const ros2_unitree_legged_msgs::msg::BmsCmd::SharedPtr connection) {this->check_connection(connection);});
        }
        
        void send_written_command(const std::string &input);

};

void Control::check_connection(const ros2_unitree_legged_msgs::msg::BmsCmd::SharedPtr connection){
    connected = (connection && connection->off != 0xA5);
}

void Control::send_written_command(const std::string &input){
    // Ignore empty commands from audio transcription
    if (input.empty() || input.find_first_not_of(" \t\n\v\f\r") == std::string::npos) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Received empty command. Ignoring.");
        return;
    }

    auto request = std::make_shared<robodog_gpt::srv::Comms::Request>();
    request->input = input;    

    while (!gpt_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = gpt_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service gpt_req");
        return;
    }
    gpt_response = result.get();
    
    gpt_command_start_time = this->now();
    gpt_command_duration = rclcpp::Duration::from_seconds(gpt_response->max_motiontime);
    
    use_gpt = true;
}

void Control::msg_callback(const std_msgs::msg::String::SharedPtr cmd){
    this->send_written_command(cmd->data);
}

void Control::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
    latest_twist = *msg;
    last_twist_time = this->now();
}

void Control::audio_callback(const std_msgs::msg::Bool::SharedPtr state){
    auto request = std::make_shared<robodog_gpt::srv::Audio::Request>();
    request->trigger = state->data;

    while (!audio_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto future = audio_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call audio service");
        return;
    }

    auto response = future.get();
    if (!state->data) {
        this->send_written_command(response->output);  
    }
}


void Control::control_loop(){
    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;
    
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.level_flag = HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gait_type = 0;
    high_cmd_ros.speed_level = 0;
    high_cmd_ros.foot_raise_height = 0;
    high_cmd_ros.body_height = 0;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yaw_speed = 0.0f;
    high_cmd_ros.reserve = 0;

    // FIX: Switched to a time-based check instead of a broken counter.
    if (use_gpt && gpt_response && (this->now() - gpt_command_start_time < gpt_command_duration)){
        high_cmd_ros.mode = gpt_response->mode;
        high_cmd_ros.gait_type = gpt_response->gait_type;
        high_cmd_ros.speed_level = gpt_response->speed_level;
        high_cmd_ros.foot_raise_height = gpt_response->foot_raise_height;
        high_cmd_ros.body_height = gpt_response->body_height;
        high_cmd_ros.position[0] = gpt_response->position[0];
        high_cmd_ros.position[1] = gpt_response->position[1];
        high_cmd_ros.euler[0] = gpt_response->euler[0];
        high_cmd_ros.euler[1] = gpt_response->euler[1];
        high_cmd_ros.euler[2] = gpt_response->euler[2];
        high_cmd_ros.velocity[0] = gpt_response->velocity[0];
        high_cmd_ros.velocity[1] = gpt_response->velocity[1];
        high_cmd_ros.yaw_speed = gpt_response->yaw_speed;
        high_cmd_ros.reserve = gpt_response->reserve;
    } else {
        use_gpt = false;
        high_cmd_ros.mode = MODE;
        high_cmd_ros.gait_type = GAIT;
        high_cmd_ros.speed_level = 0;
        high_cmd_ros.foot_raise_height = FOOT_HEIGHT;

        if ((this->now() - last_twist_time).seconds() > 0.5) { // FIX: Reduced timeout for safety
            latest_twist = geometry_msgs::msg::Twist();
        }

        if (latest_twist.linear.x > 0){
            high_cmd_ros.velocity[0] = FORWARD_VEL;
        } else if (latest_twist.linear.x < 0) {
            high_cmd_ros.velocity[0] = BACKWARD_VEL;
        }

        if (latest_twist.linear.y > 0){
            high_cmd_ros.velocity[1] = RIGHT_STRAFE_VEL;
        } else if (latest_twist.linear.y < 0) {
            high_cmd_ros.velocity[1] = LEFT_STRAFE_VEL;
        }

        if (latest_twist.angular.z > 0){
            high_cmd_ros.yaw_speed = ROTATE_RIGHT;
        } else if (latest_twist.angular.z < 0){
            high_cmd_ros.yaw_speed = ROTATE_LEFT;
        }
    }
    
    if (connected) pub->publish(high_cmd_ros);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Control>();
    // FIX: Use a MultiThreadedExecutor to prevent service calls from blocking other callbacks.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}