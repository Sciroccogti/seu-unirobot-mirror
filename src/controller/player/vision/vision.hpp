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
#include "math/math.hpp"
#include "sensor/imu.hpp"
#include "sensor/motor.hpp"
#include "imageproc/detector/detector.h"

class Vision: public timer, public subscriber, public singleton<Vision>
{
public:
    Vision();
    ~Vision();
    int w(){return w_;}
    int h(){return h_;}
    void updata(const pub_ptr &pub, const int &type);
    bool start();
    void stop();
    void set_img_send_type(image_send_type t)
    {
        img_sd_type_ = t;
    }

    void set_camera_info(const camera_info &para);

private:
    Eigen::Vector2d odometry(const Eigen::Vector2i &pos, const Eigen::Vector3d &vec);
private:
    void run();
    void send_image(const cv::Mat &src);
    
    imu::imu_data imu_data_;
    float head_yaw_;

    bool use_mv_;
    int p_count_;
    int w_, h_;
    int camera_w_,  camera_h_, camera_size_;
    std::map<std::string, camera_info> camera_infos_;
    std::map<int, float> dets_prob_;
    std::vector<detection> ball_dets_;
    std::vector<detection> post_dets_;


    bool is_busy_;
    image_send_type img_sd_type_;

    network net_;
    unsigned char *dev_src_;
    unsigned char *dev_bgr_;
    unsigned char *dev_undistored_;
    unsigned char *dev_ori_;
    unsigned char *dev_sized_;
    unsigned char *dev_yuyv_;
    unsigned char *camera_src_;
    float *dev_rgbfp_;
    int src_size_;
    int bgr_size_;
    int ori_size_;
    int yuyv_size_;
    int sized_size_;
    int rgbf_size_;
    std::vector<std::string> names_;

    object_prob ball_, post_;
    int cant_see_ball_count_;
    camera_param params_;
    Eigen::Vector3d camera_vec_;

    std::shared_ptr<vision::Detector> detector_;

    mutable std::mutex frame_mtx_, imu_mtx_;
};

#define VISION Vision::instance()

