#include "rclcpp/rclcpp.hpp"
#include "robodog_gpt/audio.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "miniaudio.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <cstdio>
#include <thread>
#include <atomic>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem;

std::string WAV_PATH = "temp.wav";

ma_encoder encoder;
ma_device device;
std::atomic<bool> recording(false);
std::thread recording_thread;

void AudioNode::data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount) {
    if (pInput) {
        ma_encoder_write_pcm_frames((ma_encoder*)pDevice->pUserData, pInput, frameCount, NULL);
    }
    (void)pOutput;
}

AudioNode::AudioNode() : rclcpp::Node("audio_node") {
    audio_pub_ = this->create_publisher<std_msgs::msg::String>("/comms", 10);
    
    auto audio_sub_ = this->create_subscription<std_msgs::msg::Bool>("/audio", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) {
                start_recording();
            } else {
                stop_recording();
            }
        });
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Audio service ready.");
}

bool AudioNode::record_setup(int sample_rate) {
    ma_encoder_config encoder_config = ma_encoder_config_init(
        ma_encoding_format_wav, 
        ma_format_s16, 
        1, 
        sample_rate
    );
    
    ma_result res = ma_encoder_init_file(WAV_PATH.c_str(), &encoder_config, &encoder);
    if (res != MA_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Failed to initialize encoder.");
        return false;
    }

    device_config_ = ma_device_config_init(ma_device_type_capture);
    device_config_.capture.format = encoder.config.format;
    device_config_.capture.channels = encoder.config.channels;
    device_config_.sampleRate = encoder.config.sampleRate;
    device_config_.dataCallback = AudioNode::data_callback;
    device_config_.pUserData = &encoder;

    res = ma_device_init(NULL, &device_config_, &device_);
    if (res != MA_SUCCESS) {
        ma_encoder_uninit(&encoder);
        RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Failed to initialize capture device.");
        return false;
    }
    return true;
}

void recording_function() {
    ma_result res = ma_device_start(&device);
    if (res != MA_SUCCESS) {
        ma_device_uninit(&device);
        ma_encoder_uninit(&encoder);
        RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Failed to start device");
        return;
    }

    while (recording) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ma_device_uninit(&device);
    ma_encoder_uninit(&encoder);
}

void AudioNode::start_recording() {
    if (recording) {
        RCLCPP_WARN(rclcpp::get_logger("audio_service"), "Already recording.");
        return;
    }
    
    if (!record_setup()) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Record setup failed. Aborting recording.");
        return;
    }
    
    recording = true;
    recording_thread = std::thread(recording_function);
    RCLCPP_INFO(rclcpp::get_logger("audio_service"), "Recording started.");
}

void AudioNode::stop_recording() {
    if (!recording) {
        RCLCPP_WARN(rclcpp::get_logger("audio_service"), "Not recording, cannot stop.");
        return;
    }
    
    recording = false;
    if (recording_thread.joinable()) {
        recording_thread.join();
    }
    
    RCLCPP_INFO(rclcpp::get_logger("audio_service"), "Recording stopped. Transcribing...");
    
    // Transcription logic (you'll need to implement or move this)
    std::string TXT_PATH = WAV_PATH + ".txt";
    std::string WHISPER_BIN = "/home/robodog/ros2_ws/src/robodog_gpt/include/whisper.cpp/build/bin/whisper-cli";
    std::string WHISPER_MODEL = "/home/robodog/ros2_ws/src/robodog_gpt/include/whisper.cpp/models/ggml-small.en.bin";
    
    std::string command = WHISPER_BIN + " -m " + WHISPER_MODEL + " -nt -otxt " + WAV_PATH;
    
    int ret_code = system(command.c_str());
    if (ret_code != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Whisper command failed with exit code %d.", ret_code);
    }
    
    std::ifstream infile(TXT_PATH);
    std::string transcription;
    if (!infile) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Failed to open transcription file: %s", TXT_PATH.c_str());
    } else {
        std::stringstream buffer;
        buffer << infile.rdbuf();
        transcription = buffer.str();
        transcription.erase(0, transcription.find_first_not_of(" \t\n\v\f\r"));
        transcription.erase(transcription.find_last_not_of(" \t\n\v\f\r") + 1);
    }
    
    std_msgs::msg::String out;
    out.data = transcription;
    audio_pub_->publish(out);
    RCLCPP_INFO(rclcpp::get_logger("audio_service"), "Transcription: '%s'", transcription.c_str());
    
    if (fs::exists(WAV_PATH)) fs::remove(WAV_PATH);
    if (fs::exists(TXT_PATH)) fs::remove(TXT_PATH);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AudioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}