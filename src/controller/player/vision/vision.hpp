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
#include "darknet/network.h"
#include "image/image_define.hpp"
#include "parser/color_parser.hpp"

class Vision: public timer, public subscriber, public singleton<Vision>
{
public:
    Vision();
    ~Vision();

    void updata(const pro_ptr &pub, const int &type);
    bool start(const sensor_ptr &s = nullptr);
    void stop();
    void caculateColor(imageproc::Color c, int x, int y, int w, int h);
    void set_img_send_type(image_send_type t)
    {
        img_sd_type_ = t;
    }
    mutable std::mutex frame_mutex_;
private:
    void run();
    void send_image(const cv::Mat &src);
    void yuyv2dst();

    std::shared_ptr<tcp_server> server_;
    int p_count_;
    std::string filename_;
    int w_, h_;
    network net_;
    bool is_busy_;
    image_send_type img_sd_type_;

    unsigned char *dev_src_;
    unsigned char *dev_bgr_;
    float *dev_rgbf_;
    float *dev_wsized_;
    float *dev_sized_;
    float *dev_hsi_;

    int src_size_;
    int bgr_size_;
    int rgbf_size_;
    int sizew_size_;
    int sized_size_;
    int hsi_size_;

    unsigned char *yuyv_;
    std::vector<std::string> names_;
    std::map<imageproc::Color, imageproc::ColorHSI> colors_;
};

#define VISION Vision::instance()

