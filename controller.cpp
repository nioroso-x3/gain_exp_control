#include <opencv2/opencv.hpp>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>
#include <vector>
#include <numeric>
#include <cstring>
#include <chrono>
#include <cmath>
#include <getopt.h>
#define MAX_EXP 6

float clamp(float val, float min_val, float max_val) {
    return std::max(std::min(val, max_val), min_val);
}

bool set_v4l2_control(const std::string& device, int id, int value) {
    struct v4l2_control control = {};
    control.id = id;
    control.value = value;

    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Opening video device");
        return false;
    }

    if (ioctl(fd, VIDIOC_S_CTRL, &control) < 0) {
        perror("Setting control");
        close(fd);
        return false;
    }

    close(fd);
    return true;
}

bool set_gain(const std::string& device, int gain) {
    return set_v4l2_control(device, V4L2_CID_GAIN, gain);
}

//change exp_map according to the valid exposure values from your camera that you want to use
bool set_exposure_absolute(const std::string& device, int index) {
    const int exp_map[] = {1, 2, 5, 10, 20, 39, 78};
    if (index < 0) index = 0;
    if (index > MAX_EXP) index = MAX_EXP;
    return set_v4l2_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, exp_map[index]);
}

int get_histogram_brightness(const cv::Mat& gray) {
    int histSize = 256;
    float range[] = { 0, 256 };
    const float* histRange = { range };
    cv::Mat hist;
    cv::calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
    std::vector<float> cumsum(histSize);
    cumsum[0] = hist.at<float>(0);
    for (int i = 1; i < histSize; ++i) {
        cumsum[i] = cumsum[i - 1] + hist.at<float>(i);
    }
    float total = gray.total();
    for (int i = 0; i < histSize; ++i) {
        if (cumsum[i] >= total / 2.0f) return i;
    }
    return 0;
}

int main(int argc, char** argv) {
    std::string pipeline, device;
    float gain_min = -1, gain_max = -1, target = 50;
    float kp = 0.1, ki = 0.001, kd = 0.1;
    bool show_help = false;

    static struct option long_options[] = {
        {"gain-min", required_argument, 0, 0},
        {"gain-max", required_argument, 0, 0},
        {"pipeline", required_argument, 0, 0},
        {"device", required_argument, 0, 0},
        {"target", required_argument, 0, 0},
        {"kp", required_argument, 0, 0},
        {"ki", required_argument, 0, 0},
        {"kd", required_argument, 0, 0},
        {"help", no_argument, 0, 0},
        {0, 0, 0, 0}
    };

    int option_index = 0;
    while (true) {
        int c = getopt_long(argc, argv, "", long_options, &option_index);
        if (c == -1) break;

        switch (option_index) {
            case 0: gain_min = std::stof(optarg); break;
            case 1: gain_max = std::stof(optarg); break;
            case 2: pipeline = optarg; break;
            case 3: device = optarg; break;
            case 4: target = std::stof(optarg); break;
            case 5: kp = std::stof(optarg); break;
            case 6: ki = std::stof(optarg); break;
            case 7: kd = std::stof(optarg); break;
            case 8: show_help = true; break;
        }
    }

    if (show_help || gain_min < 0 || gain_max < 0 || pipeline.empty() || device.empty()) {
        std::cout << "Usage: " << argv[0] << " \\\n"
                  << "  --gain-min <int>       Minimum gain value\n"
                  << "  --gain-max <int>       Maximum gain value\n"
                  << "  --pipeline <string>    GStreamer pipeline for OpenCV\n"
                  << "  --device <string>      V4L2 device path (e.g., /dev/video0)\n"
                  << "  [--target <int>]       Target brightness (0-255, default: 50)\n"
                  << "  [--kp <float>]         Proportional gain (default: 0.1)\n"
                  << "  [--ki <float>]         Integral gain (default: 0.001)\n"
                  << "  [--kd <float>]         Derivative gain (default: 0.1)\n"
                  << "  [--help]               Show this help message\n";
        return 0;
    }

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video stream" << std::endl;
        return 1;
    }

    float exposure = MAX_EXP / 2.0;
    float gain = (gain_max - gain_min) / 2.0;
    float previous_error = 0;
    float integral = 0;
    auto last_time = std::chrono::steady_clock::now();
    std::vector<int> brightness_vals;
    uint64_t frames = 0;
    std::cout << "Starting PID gain control loop..." << std::endl;
    std::cout << "G: " << gain_min << "/" << gain_max << " Kp: " << kp << " ki: " << ki << " kd: " << kd << "\n"; 
    while (true) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            std::cerr << "Frame capture failed" << std::endl;
            break;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        float brightness = get_histogram_brightness(gray);
        brightness_vals.push_back(brightness);
        if (brightness_vals.size() < 2) continue;
        if (brightness_vals.size() > 2) brightness_vals.erase(brightness_vals.begin());
        float avg_brightness = std::accumulate(brightness_vals.begin(), brightness_vals.end(), 0.0f) / brightness_vals.size();

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();

        float error = target - avg_brightness;
        integral += error * dt;
        float derivative = (error - previous_error) / dt;
        float output = kp * error + ki * integral + kd * derivative;

        gain += output;
        gain = clamp(gain, gain_min, gain_max);

        if (gain == gain_max || gain == gain_min) {
            if (output != 0) {
                exposure += output * float(MAX_EXP) / (gain_max - gain_min);
            }
        }
        exposure = clamp(exposure, 0, MAX_EXP);

        set_gain(device, static_cast<int>(gain));
        set_exposure_absolute(device, exposure);
        //if(frames % 5 == 0)
        //  std::cout << "O:" << output << " E: " << error << " I: " << integral << " D: " << derivative << " gain: " << gain << " exp: " << exposure << "\n";
        previous_error = error;
        last_time = now;
        frames++;
    }

    cap.release();
    return 0;
}
