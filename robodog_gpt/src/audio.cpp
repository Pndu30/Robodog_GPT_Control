#include "rclcpp/rclcpp.hpp"
#include "robodog_gpt/srv/audio.hpp"
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

std::string WAV_PATH = "temp.wav";
std::string TXT_PATH = "temp_wav.txt"; 

namespace fs = std::filesystem;
std::atomic<bool> recording(false);
std::thread recording_thread;

ma_encoder encoder;
ma_device device;

void data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount){
    ma_encoder_write_pcm_frames((ma_encoder*)pDevice->pUserData, pInput, frameCount, NULL);
    (void)pOutput;
}

bool record_setup(int sample_rate=44100){
    ma_encoder_config encoder_config = ma_encoder_config_init(ma_encoding_format_wav, ma_format_s16, 1, sample_rate);
    ma_result res = ma_encoder_init_file(WAV_PATH.c_str(), &encoder_config, &encoder);
    if (res != MA_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Failed to initialize encoder.");
        return false;
    }

    ma_device_config device_config;
    device_config = ma_device_config_init(ma_device_type_capture);
    device_config.capture.format   = encoder.config.format;
    device_config.capture.channels = encoder.config.channels;
    device_config.sampleRate       = encoder.config.sampleRate;
    device_config.dataCallback     = data_callback;
    device_config.pUserData        = &encoder;

    res = ma_device_init(NULL, &device_config, &device);
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

    while(recording){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ma_device_uninit(&device);
    ma_encoder_uninit(&encoder);
}


std::string postprocess(){
    std::string WHISPER_BIN = "/home/robodog/ros2_ws/src/robodog_gpt/include/whisper.cpp/build/bin/whisper-cli";
    std::string WHISPER_MODEL = "/home/robodog/ros2_ws/src/robodog_gpt/include/whisper.cpp/models/ggml-small.en.bin";

    std::string command = WHISPER_BIN + " -m " + WHISPER_MODEL + " -nt -otxt " + WAV_PATH;
    
    int ret_code = system(command.c_str());
    if (ret_code != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Whisper command failed with exit code %d.", ret_code);
    }

    std::ifstream infile(TXT_PATH);
    std::string out; 
    if (!infile) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Failed to open transcription file: %s", TXT_PATH.c_str());
    } else {
        std::stringstream buffer;
        buffer << infile.rdbuf();
        out = buffer.str();
        out.erase(0, out.find_first_not_of(" \t\n\v\f\r"));
        out.erase(out.find_last_not_of(" \t\n\v\f\r") + 1);
    }
    return out;
}

void audio_req(const std::shared_ptr<robodog_gpt::srv::Audio::Request> request, 
                std::shared_ptr<robodog_gpt::srv::Audio::Response> response){
    if (request->trigger) {
        if (recording) {
             RCLCPP_WARN(rclcpp::get_logger("audio_service"), "Already recording.");
            return;
        }
        
        // FIX: Moved record_setup() here to initialize before every recording.
        if (!record_setup()) {
            RCLCPP_ERROR(rclcpp::get_logger("audio_service"), "Record setup failed. Aborting recording.");
            return;
        }
        
        recording = true;
        recording_thread = std::thread(recording_function);
        RCLCPP_INFO(rclcpp::get_logger("audio_service"), "Recording started.");
    } else {
        if (!recording) {
            RCLCPP_WARN(rclcpp::get_logger("audio_service"), "Not recording, cannot stop.");
            return;
        }
        recording = false;
        if (recording_thread.joinable()) {
            recording_thread.join();
        }
        RCLCPP_INFO(rclcpp::get_logger("audio_service"), "Recording stopped. Transcribing...");

        std::string transcription = postprocess();
        response->output = transcription;
        RCLCPP_INFO(rclcpp::get_logger("audio_service"), "Transcription: '%s'", transcription.c_str());

        if (fs::exists(WAV_PATH)) fs::remove(WAV_PATH);
        if (fs::exists(TXT_PATH)) fs::remove(TXT_PATH);
    }
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("audio_service");
    rclcpp::Service<robodog_gpt::srv::Audio>::SharedPtr sv = node->create_service<robodog_gpt::srv::Audio>("/audio", &audio_req);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Audio service ready.");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}