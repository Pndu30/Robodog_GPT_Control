#include "rclcpp/rclcpp.hpp"
#include "openai/openai.hpp"
#include "nlohmann/json.hpp"
#include <string>
#include "robodog_gpt/srv/comms.hpp"
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

void gpt_req(
    const std::shared_ptr<robodog_gpt::srv::Comms::Request> request,
    std::shared_ptr<robodog_gpt::srv::Comms::Response> response)
{
    std::string command = std::string(COMMAND) + request->input;
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

        response->mode = safe_get<int>(j, "mode", 0);
        response->gait_type = safe_get<int>(j, "gait_type", 0);
        response->speed_level = safe_get<float>(j, "speed_level", 0.0f);
        response->foot_raise_height = safe_get<float>(j, "foot_raise_height", 0.0f);
        response->body_height = safe_get<float>(j, "body_height", 0.0f);
        response->position[0] = safe_get<float>(j, "position0", 0.0f);
        response->position[1] = safe_get<float>(j, "position1", 0.0f);
        response->euler[0] = safe_get<float>(j, "euler0", 0.0f);
        response->euler[1] = safe_get<float>(j, "euler1", 0.0f);
        response->euler[2] = safe_get<float>(j, "euler2", 0.0f);
        response->velocity[0] = safe_get<float>(j, "velocity0", 0.0f);
        response->velocity[1] = safe_get<float>(j, "velocity1", 0.0f);
        response->yaw_speed = safe_get<float>(j, "yaw_speed", 0.0f);
        response->reserve = safe_get<int>(j, "reserve", 0);
        response->max_motiontime = safe_get<float>(j, "max_motiontime", 2.0f);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "An exception occurred during OpenAI request: %s", e.what());
    } 
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const char* api_key = std::getenv("OPENAI_API_KEY");
    if (api_key == nullptr) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),
                     "OPENAI_API_KEY environment variable not set. Exiting.");
        return 1;
    }
    openai::start(api_key);

    auto node = rclcpp::Node::make_shared("gpt_service");
    auto service = node->create_service<robodog_gpt::srv::Comms>("/comms", &gpt_req);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GPT service ready.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
