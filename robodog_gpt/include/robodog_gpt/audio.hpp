#ifndef ROBODOG_GPT_AUDIO_HPP
#define ROBODOG_GPT_AUDIO_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <miniaudio.h>

class AudioNode : public rclcpp::Node {
public:
    AudioNode();

    bool record_setup(int sample_rate = 44100);
    void start_recording();
    void stop_recording();

private:
    static void data_callback(
        ma_device* pDevice,
        void* pOutput,
        const void* pInput,
        ma_uint32 frameCount
    );

    ma_device device_;
    ma_device_config device_config_;
    bool recording_ = false;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr audio_pub_;
};

#endif