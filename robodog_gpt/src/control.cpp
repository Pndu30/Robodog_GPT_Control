#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "convert.h"
#include <string>
#include <functional>
#include "geometry_msgs/msg/twist.hpp"
#include "robodog_gpt/srv/comms.hpp"
#include "std_msgs/msg/string.hpp"

#define MODE 2
#define GAIT 2
#define FOOT_HEIGHT 0.1
#define FORWARD_VEL 0.4f
#define BACKWARD_VEL -0.4f
#define RIGHT_STRAFE_VEL 0.4f
#define LEFT_STRAFE_VEL -0.4f
#define ROTATE_RIGHT 2.0f
#define ROTATE_LEFT -2.0f

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;

class Control : public rclcpp::Node{
    private:
        rclcpp::Client<robodog_gpt::srv::Comms>::SharedPtr gpt_client;
        rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub;
        rclcpp::TimerBase::SharedPtr timer;
        
        bool use_gpt = false;
        long motiontime = 0;
        rclcpp::Time last_twist_time;
        geometry_msgs::msg::Twist latest_twist;
        robodog_gpt::srv::Comms::Response::SharedPtr response;

        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void cmd_callback(const std_msgs::msg::String::SharedPtr cmd);
        void control_loop();
    

    public:
        Control() : Node("control_node"){
            gpt_client = this->create_client<robodog_gpt::srv::Comms>("/comms");
            pub = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 10);
            twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("control", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {this->twist_callback(msg);});
            cmd_sub = this->create_subscription<std_msgs::msg::String>("cmd", 10, [this](const std_msgs::msg::String::SharedPtr cmd) {this->cmd_callback(cmd);});
            timer = this->create_wall_timer(2s, std::bind(&Control::control_loop, this));
            last_twist_time = this->now();
        }
        
        void send_written_command(const std::string &input);

};

void Control::send_written_command(const std::string &input){
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
    response = result.get();
    use_gpt = true;
    motiontime = 0;
}

void Control::cmd_callback(const std_msgs::msg::String::SharedPtr cmd){
    this->send_written_command(cmd->data);
}

void Control::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
    latest_twist = *msg;
    last_twist_time = this->now();
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

    if (use_gpt && response && motiontime < response->max_motiontime){
        motiontime += 2;
        high_cmd_ros.mode = response->mode;
        high_cmd_ros.gait_type = response->gait_type;
        high_cmd_ros.speed_level = response->speed_level;
        high_cmd_ros.foot_raise_height = response->foot_raise_height;
        high_cmd_ros.body_height = response->body_height;
        high_cmd_ros.position[0] = response->position[0];
        high_cmd_ros.position[1] = response->position[1];
        high_cmd_ros.euler[0] = response->euler[0];
        high_cmd_ros.euler[1] = response->euler[1];
        high_cmd_ros.euler[2] = response->euler[2];
        high_cmd_ros.velocity[0] = response->velocity[0];
        high_cmd_ros.velocity[1] = response->velocity[1];
        high_cmd_ros.yaw_speed = response->yaw_speed;
        high_cmd_ros.reserve = response->reserve;
    } else {
        use_gpt = false;
        high_cmd_ros.mode = MODE;
        high_cmd_ros.gait_type = GAIT;
        high_cmd_ros.speed_level = 0;
        high_cmd_ros.foot_raise_height = FOOT_HEIGHT;

        if ((this->now() - last_twist_time).seconds() > 1.0) {
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
    
    pub->publish(high_cmd_ros);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
