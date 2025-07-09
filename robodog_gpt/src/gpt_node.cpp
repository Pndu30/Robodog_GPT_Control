#include "rclcpp/rclcpp.hpp"
#include "openai/openai.hpp"
#include "nlohmann/json.hpp"
#include <string>
#include "robodog_gpt/srv/comms.hpp"

#define API_KEY "test_api"
#define COMMAND \
"Given variables to change for the unitree GO1 robodog based on the given unitree_ros2_to_real library, " \
"JUST RETURN A JSON OBJECT with keys: mode, gait_type, speed_level, foot_raise_height, body_height, " \
"position0, position1, euler0, euler1, euler2, velocity0, velocity1, yaw_speed, reserve. " \
"No explanations or extra text. Respond only with JSON." \
"Command: "

void gpt_req(const std::shared_ptr<robodog_gpt::srv::Comms::Request> request, 
            std::shared_ptr<robodog_gpt::srv::Comms::Response> response) {
    std::string command = std::string(COMMAND) + request->input;
    auto out = openai::chat().create({
            {"model", "gpt-3.5-turbo"},
            {"messages", {
                {{"role", "user"}, {"content", command}}
            }},
            {"max_tokens", 128},
            {"temperature", 0}
    });

    std::string reply = out["choices"][0]["message"]["content"];
    auto j = nlohmann::json::parse(reply);

    response->mode = j["mode"];
    response->gait_type = j["gait_type"];
    response->speed_level = j["speed_level"];
    response->foot_raise_height = j["foot_raise_height"];
    response->body_height = j["body_height"];
    response->position[0] = j["position0"];
    response->position[1] = j["position1"];
    response->euler[0] = j["euler0"];
    response->euler[1] = j["euler1"];
    response->euler[2] = j["euler2"];
    response->velocity[0] = j["velocity0"];
    response->velocity[1] = j["velocity1"];
    response->yaw_speed = j["yaw_speed"];
    response->reserve = j["reserve"];
    response->max_motiontime = j["max_motiontime"];

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: %s", request->input);                                         
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %s", (string)response->output);
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    openai::start(API_KEY);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gpt_service");
    rclcpp::Service<robodog_gpt::srv::Comms>::SharedPtr sv = node->create_service<robodog_gpt::srv::Comms>("/comms", &gpt_req);
    
    rclcpp::spin(node);    
    rclcpp::shutdown();
    return 0;
}

