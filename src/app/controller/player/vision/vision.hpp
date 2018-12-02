#pragma once

#include <mutex>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "timer.hpp"
#include "pattern.hpp"
#include "sensor/camera.hpp"
#include "configuration.hpp"
#include "options/options.hpp"
#include "common.hpp"
#include "singleton.hpp"
#include "darknet/network.h"
#include "tcp.hpp"
#include "common.hpp"

class Vision: public timer, public subscriber, public singleton<Vision>
{
public:
    Vision();
    ~Vision();

    void updata(const pro_ptr &pub, const int &type);
    bool start();
    void stop();
    void set_img_send_type(image_send_type t)
    {
        img_sd_type_ = t;
    }

    void set_camera_para(const camera_para &para);
    mutable std::mutex frame_mutex_;
private:
    void run();
    void send_image(const cv::Mat &src);
    void src2dst();

    bool use_mv_;
    int p_count_;
    std::string filename_;
    int w_, h_;
    int camera_w_,  camera_h_, camera_size_;
    std::map<std::string, camera_para> camera_infos_;

    network net_;
    bool is_busy_;
    image_send_type img_sd_type_;

    unsigned char *dev_src_;
    unsigned char *dev_bgr_;
    unsigned char *dev_ori_;
    unsigned char *dev_sized_;

    float *dev_rgbfp_;

    int src_size_;
    int bgr_size_;
    int ori_size_;
    int sized_size_;

    int rgbf_size_;
    unsigned char *camera_src_;
    std::vector<std::string> names_;
};

#define VISION Vision::instance()

