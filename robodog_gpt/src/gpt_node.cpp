#include "rclcpp/rclcpp.hpp"
#include "openai/openai.hpp"
#include "nlohmann/json.hpp"
#include <string>
#include "robodog_gpt/srv/comms.hpp"
#include <cstdlib>

#define COMMAND \
"Given variables to change for the unitree GO1 robodog based on the given unitree_ros2_to_real library, " \
"JUST RETURN A JSON OBJECT with keys: mode, gait_type, speed_level, foot_raise_height, body_height, " \
"position0, position1, euler0, euler1, euler2, velocity0, velocity1, yaw_speed, reserve, and max_motiontime. " \
"max_motiontime should be a float representing the duration of the action in seconds (e.g., 3.5). " \
"No explanations or extra text. Respond only with JSON." \
"Command: "

void gpt_req(const std::shared_ptr<robodog_gpt::srv::Comms::Request> request, 
            std::shared_ptr<robodog_gpt::srv::Comms::Response> response) {
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

        auto j = nlohmann::json::parse(reply, nullptr, false);
        if (j.is_discarded()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to parse JSON response from OpenAI: %s", reply.c_str());
            return;
        }
        
        // Use .value() for safe access with default values
        response->mode = j.value("mode", 0);
        response->gait_type = j.value("gait_type", 0);
        response->speed_level = j.value("speed_level", 0.0f);
        response->foot_raise_height = j.value("foot_raise_height", 0.0f);
        response->body_height = j.value("body_height", 0.0f);
        response->position[0] = j.value("position0", 0.0f);
        response->position[1] = j.value("position1", 0.0f);
        response->euler[0] = j.value("euler0", 0.0f);
        response->euler[1] = j.value("euler1", 0.0f);
        response->euler[2] = j.value("euler2", 0.0f);
        response->velocity[0] = j.value("velocity0", 0.0f);
        response->velocity[1] = j.value("velocity1", 0.0f);
        response->yaw_speed = j.value("yaw_speed", 0.0f);
        response->reserve = j.value("reserve", 0);
        response->max_motiontime = j.value("max_motiontime", 2.0f);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "An exception occurred during OpenAI request: %s", e.what());
    }
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    const char* api_key = std::getenv("OPENAI_API_KEY");
    if (api_key == nullptr) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "OPENAI_API_KEY environment variable not set. Exiting.");
        return 1;
    }
    openai::start(api_key);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gpt_service");
    rclcpp::Service<robodog_gpt::srv::Comms>::SharedPtr sv = node->create_service<robodog_gpt::srv::Comms>("/comms", &gpt_req);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GPT service ready.");
    rclcpp::spin(node);    
    rclcpp::shutdown();
    return 0;
}