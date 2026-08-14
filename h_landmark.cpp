#include <iostream>
#include <opencv2/opencv.hpp>
#include <gpiod.hpp>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <array>
#include <cmath>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include "nadjieb/mjpeg_streamer.hpp"
#include "pca9685.hpp"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define IP_STREAM_URL "http://admin:admin@192.168.1.62:8081/video"
#define ALPHA         0.2f

bool system_running = true;
std::mutex state_mutex;


struct Vec3D {
    float x, y, z;

    Vec3D operator-(const Vec3D& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    float dot(const Vec3D& other) const {
        return (x * other.x) + (y * other.y) + (z * other.z);
    }

    Vec3D cross(const Vec3D& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }
    float mag() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    Vec3D normalized() const {
        float m = mag();
        if (m < 0.0001f) return {0, 0, 0};
        return {x / m, y / m, z / m};
    }
};

struct HandState {
    std::array<float, 5> fingers;

    HandState operator-(const HandState& other) const {
        HandState result;
        for (int i = 0; i < 5; ++i) result.fingers[i] = fingers[i] - other.fingers[i];
        return result;
    }
    HandState operator+(const HandState& other) const {
        HandState result;
        for (int i = 0; i < 5; ++i) result.fingers[i] = fingers[i] + other.fingers[i];
        return result;
    }
    HandState operator*(float alpha) const {
        HandState result;
        for (int i = 0; i < 5; ++i) result.fingers[i] = fingers[i] * alpha;
        return result;
    }
};

HandState global_goal_state = {0, 0, 0, 0, 0};

class Camera {
    private: 
        std     ::string landmark_model_path = "hand_landmark_full.tflite"                 ;
        std     ::       unique_ptr          <tflite::FlatBufferModel> landmark_model      ;
        std     ::       unique_ptr          <tflite::Interpreter> landmark_interpreter    ;
        std     ::string stream_url          = IP_STREAM_URL                               ;
        cv      ::       VideoCapture        cap                                           ;
        cv      ::       Mat                 frame, rgb_frame, resized_frame               ;
        std     ::       vector              <uchar> buff                                  ;
        std     ::       vector              <int > params = {cv::IMWRITE_JPEG_QUALITY, 80};
        nadjieb ::       MJPEGStreamer       streamer                                      ;
    public:
        bool cam_init = true;

        Camera() {
            landmark_model = tflite::FlatBufferModel::BuildFromFile(landmark_model_path.c_str());

            if (!landmark_model) {
                std::cerr << "AI Failed to Init\n";
                cam_init = false;
            } else {
                tflite::ops::builtin::BuiltinOpResolver resolver;
                tflite::InterpreterBuilder(*landmark_model, resolver)(&landmark_interpreter);
                
                landmark_interpreter->AllocateTensors();
                std::cout << "AI Init Success\n";
            }

            cap.open(stream_url);
            cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

            if (!cap.isOpened()) {
                std::cerr << "Cam Failed to Init\n";
                cam_init = false;
            } else {
                std::cout << "Cam Init Success\n";
            }
            
            for(int i = 0; i < 5; i++) {
                cap.read(frame);
            }

            streamer.start(8080);
            std::cout << "Stream Init Success\n";
        }

        ~Camera() {
            cap.release();
            streamer.stop();
        }
    
        void hand_det() { 
            while (system_running && cam_init) {
                cap.read(frame);
                if (frame.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue; 
                }
                
                cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);
                
                cv::resize(rgb_frame, resized_frame, cv::Size(224, 224));
                
                if (!resized_frame.isContinuous()) {
                    resized_frame = resized_frame.clone(); 
                }

                float* input_tensor = landmark_interpreter->typed_input_tensor<float>(0);
                uint8_t* in_data = resized_frame.ptr<uint8_t>(0);
                int num_pixels = 224 * 224 * 3;
                
                for (int i = 0; i < num_pixels; ++i) {
                    input_tensor[i] = (in_data[i] / 255.0f); 
                }

                if (landmark_interpreter->Invoke() == kTfLiteOk) {

                    float* landmarks = landmark_interpreter->typed_output_tensor<float>(0);

                    auto get_pt = [&](int index) -> Vec3D {
                        return {landmarks[index * 3], landmarks[index * 3 + 1], landmarks[index * 3 + 2]};
                    };

                    auto calc_servo = [](const Vec3D& p1, const Vec3D& p2, const Vec3D& p3) -> float {
                        Vec3D boneA = p1 - p2;
                        Vec3D boneB = p3 - p2;
                        
                        float magA = boneA.mag();
                        float magB = boneB.mag();
                        if (magA < 0.0001f || magB < 0.0001f) return 0.0f; 
                        
                        float dot_prod = boneA.dot(boneB);
                        float cos_theta = std::max(-1.0f, std::min(1.0f, dot_prod / (magA * magB)));
                        float angle_deg = std::acos(cos_theta) * (180.0f / M_PI);

                        float clamped = std::max(70.0f, std::min(170.0f, angle_deg));
                        return ((clamped - 70.0f) * 180.0f) / (170.0f - 70.0f);
                    };

                    HandState new_goal;
                    new_goal.fingers[0] =  calc_servo(get_pt(2 ), get_pt(3 ), get_pt(4 )) - 50;
                    new_goal.fingers[1] =  calc_servo(get_pt(5 ), get_pt(6 ), get_pt(7 ));
                    new_goal.fingers[2] =  calc_servo(get_pt(9 ), get_pt(10), get_pt(11));
                    new_goal.fingers[3] = (calc_servo(get_pt(13), get_pt(14), get_pt(15)) + 
                                                 calc_servo(get_pt(17), get_pt(18), get_pt(19)))/2.0f;
                    new_goal.fingers[4] =  calc_servo(get_pt(2 ), get_pt(3 ), get_pt(4 )) - 50;

                    {
                        std::lock_guard<std::mutex> lock(state_mutex);
                        global_goal_state = new_goal;
                    }

                    std::cout << "\rT:"       << (int) new_goal.fingers[0]
                              << " I:"        << (int) new_goal.fingers[1]
                              << " M:"        << (int) new_goal.fingers[2]
                              << " R+P:"        << (int) new_goal.fingers[3]
                              << " TOP:"        << (int) new_goal.fingers[4]
                              << "          " <<  std::flush   ;

                    float scale_x = static_cast<float>(frame.cols) / 224.0f;
                    float scale_y = static_cast<float>(frame.rows) / 224.0f;

                    std::vector<cv::Point> scaled_joints(21);
                    for (int i = 0; i < 21; ++i) {
                        Vec3D pt3d = get_pt(i);
                        
                        scaled_joints[i] = cv::Point(static_cast<int>(pt3d.x * scale_x), 
                                                    static_cast<int>(pt3d.y * scale_y));
                        
                        cv::circle(frame, scaled_joints[i], 5, cv::Scalar(0, 255, 0), cv::FILLED);
                    }

                    std::vector<std::pair<int, int>> bones = {
                        {0, 1}, {1, 2}, {2, 3}, {3, 4},         // Thumb
                        {0, 5}, {5, 6}, {6, 7}, {7, 8},         // Index
                        {5, 9}, {9, 10}, {10, 11}, {11, 12},    // Middle
                        {9, 13}, {13, 14}, {14, 15}, {15, 16},  // Ring
                        {13, 17}, {17, 18}, {18, 19}, {19, 20}, // Pinky
                        {0, 17}                                 // Palm base
                    };

                    for (const auto& bone : bones) {
                        cv::line(frame, scaled_joints[bone.first], scaled_joints[bone.second], cv::Scalar(255, 0, 0), 2);
                    }
                    cv::imencode(".jpg", frame, buff, params);
                    streamer.publish("/stream", std::string(buff.begin(), buff.end()));
                }
            }
        }
};

void ServoSmoothing(PCA9685 &driver) {
    HandState local_goal_state;
    HandState current_state;
    while (system_running) {
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            local_goal_state = global_goal_state;
        }

        current_state = current_state + (local_goal_state - current_state) * ALPHA;

        for (int i = 0; i < 5; i++) {
            driver.setAngle(i, current_state.fingers[i]);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    }
}


int main() {
    
    Camera my_cam;

    std::thread vision_thread(&Camera::hand_det, &my_cam);
    vision_thread.detach();

    PCA9685 driver1;
    driver1.setPWMFreq(50.0f);

    std::thread servo_smoothing_thread(ServoSmoothing, std::ref(driver1));

    servo_smoothing_thread.join();

    return 0;
}