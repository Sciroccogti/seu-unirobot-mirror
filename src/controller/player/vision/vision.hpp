#pragma once

#include <mutex>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "timer.hpp"
#include "pattern.hpp"
#include "sensor/camera.hpp"
#include "sensor/server.hpp"
#include "configuration.hpp"
#include "image/image_process.hpp"
#include "options/options.hpp"
#include "common.hpp"
#include "singleton.hpp"
#include "darknet/darknet.h"

class Vision: public timer, public subscriber, public singleton<Vision>
{
public:
    Vision();
    ~Vision();

    void updata(const pro_ptr &pub, const int &type);
    bool start(const sensor_ptr &s = nullptr);
    void stop();
    mutable std::mutex frame_mutex_;
private:
    void run();
    void send_image(const cv::Mat &yuvsrc);
    cv::Mat frame_;
    std::shared_ptr<tcp_server> server_;
    int p_count_;
    std::string filename_;
    network *net_;
};

#define VISION Vision::instance()

