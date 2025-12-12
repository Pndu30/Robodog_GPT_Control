#include "rclcpp/rclcpp.hpp"
#include "openai/openai.hpp"
#include "nlohmann/json.hpp"
#include <string>
#include "std_msgs/msg/string.hpp"
#include "robodog_gpt/msg/comms.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <functional>
#include <cstdlib>
#include <type_traits>

#define COMMAND \
"Given variables to change for the unitree GO1 robodog based on the given unitree_ros2_to_real library, " \
"JUST RETURN A VALID JSON OBJECT with keys: mode, gait_type, speed_level, foot_raise_height, body_height, " \
"position0, position1, euler0, euler1, euler2, velocity0, velocity1, yaw_speed, reserve, and max_motiontime. " \
"All values must be numbers (integers or floats). No strings, no explanations, no extra text. " \
"max_motiontime must be a float representing the duration in seconds (e.g., 3.5). " \
"The standard mode is 2, standard gait is 2, standard foot height is 0.1. " \
"Forward motion is velocity0 > 0 (e.g., 0.4), backward is velocity0 < 0 (e.g., -0.4). " \
"Right strafe is velocity1 > 0 (e.g., 0.4), left strafe is velocity1 < 0 (e.g., -0.4). " \
"Rotate right is yaw_speed > 0 (e.g., 2.0), rotate left is yaw_speed < 0 (e.g., -2.0). " \
"Do not output text outside of JSON. " \
"Command: "

template<typename T>
T safe_get(const nlohmann::json& j, const std::string& key, T default_value) {
    if (!j.contains(key)) return default_value;
    try {
        if (j[key].is_number()) {
            return j[key].get<T>();
        } else if (j[key].is_string()) {
            // convert string -> number
            if constexpr (std::is_same<T, int>::value) {
                return std::stoi(j[key].get<std::string>());
            } else if constexpr (std::is_same<T, float>::value) {
                return std::stof(j[key].get<std::string>());
            } else if constexpr (std::is_same<T, double>::value) {
                return std::stod(j[key].get<std::string>());
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to parse key '%s': %s. Using default.", key.c_str(), e.what());
    }
    return default_value;
}


class GPTNode : public rclcpp::Node{
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gpt_sub_;
        rclcpp::Publisher<robodog_gpt::msg::Comms>::SharedPtr gpt_pub_;
        robodog_gpt::msg::Comms comms;

        void gpt_req(const std_msgs::msg::String::SharedPtr msg);

    public:
            GPTNode() : Node("gpt_node"){
                const char* api_key = std::getenv("OPENAI_API_KEY");
                if (api_key == nullptr) {
                    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),
                                "OPENAI_API_KEY environment variable not set. Exiting.");
                }
                openai::start(api_key);
                
                gpt_pub_ = this->create_publisher<robodog_gpt::msg::Comms>("/control_gpt", 10);
                gpt_sub_ = this->create_subscription<std_msgs::msg::String>("/comms", 10, std::bind(&GPTNode::gpt_req, this, std::placeholders::_1));
                
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GPT service ready.");
            };
};

void GPTNode::gpt_req(const std_msgs::msg::String::SharedPtr msg) {
    std::string command = std::string(COMMAND) + msg->data;

    try {
        auto out = openai::chat().create({
            {"model", "gpt-3.5-turbo"},
            {"messages", {
                {{"role", "user"}, {"content", command}}
            }},
            {"max_tokens", 256},
            {"temperature", 0}
        });

        std::string reply = out["choices"][0]["message"]["content"];
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raw GPT reply: %s", reply.c_str());

        auto j = nlohmann::json::parse(reply, nullptr, false);
        if (j.is_discarded()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to parse JSON response from OpenAI.");
            return;
        }

        comms.mode = safe_get<int>(j, "mode", 0);
        comms.gait_type = safe_get<int>(j, "gait_type", 0);
        comms.speed_level = safe_get<float>(j, "speed_level", 0.0f);
        comms.foot_raise_height = safe_get<float>(j, "foot_raise_height", 0.0f);
        comms.body_height = safe_get<float>(j, "body_height", 0.0f);
        comms.position[0] = safe_get<float>(j, "position0", 0.0f);
        comms.position[1] = safe_get<float>(j, "position1", 0.0f);
        comms.euler[0] = safe_get<float>(j, "euler0", 0.0f);
        comms.euler[1] = safe_get<float>(j, "euler1", 0.0f);
        comms.euler[2] = safe_get<float>(j, "euler2", 0.0f);
        comms.velocity[0] = safe_get<float>(j, "velocity0", 0.0f);
        comms.velocity[1] = safe_get<float>(j, "velocity1", 0.0f);
        comms.yaw_speed = safe_get<float>(j, "yaw_speed", 0.0f);
        comms.reserve = safe_get<int>(j, "reserve", 0);
        comms.max_motiontime = safe_get<float>(j, "max_motiontime", 2.0f);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "An exception occurred during OpenAI request: %s", e.what());
    } 

    gpt_pub_->publish(comms);

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
