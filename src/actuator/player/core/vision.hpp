#ifndef SEU_UNIROBOT_ACTUATOR_VISION_HPP
#define SEU_UNIROBOT_ACTUATOR_VISION_HPP

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

class Vision: public timer, public subscriber, public singleton<Vision>
{
public:
    Vision():timer(CONF->get_config_value<int>("vision_period"))
    {
        p_count_ = 0;
        filename_ = get_time()+".yuv";
    }
    ~Vision()
    {
        std::cout<<"\033[32malgorithm:[Vision]   end!\033[0m\n";
    }

    void updata(const pro_ptr &pub, const int &type)
    {
        std::shared_ptr<camera> sptr = std::dynamic_pointer_cast<camera>(pub);
        if(type == sensor::SENSOR_CAMERA)
        {
            frame_mutex_.lock();
            frame_ = image::image_process::buff2yuv_mat(sptr->buffer(), sptr->buff_info());
            frame_mutex_.unlock();
        }
    }
    
    bool start(const sensor_ptr &s=nullptr)
    {
        server_ = std::dynamic_pointer_cast<tcp_server>(s);
        is_alive_ = true;
        start_timer();
        return true;
    }
    void stop()
    {
        if(is_alive_) delete_timer();
        is_alive_ = false;
    }
    mutable std::mutex frame_mutex_;
private:
    void run()
    {
        if(is_alive_)
        {
            p_count_ ++;
            frame_mutex_.lock();
            cv::Mat yuv(frame_);
            frame_mutex_.unlock();
            if(OPTS->image_record())
                image::image_process::save_yuv(yuv, filename_, std::ios::app);
            if(OPTS->use_debug())
                send_image(yuv);
        }
    }
    void send_image(const cv::Mat &yuvsrc)
    {
        cv::Mat bgr;
        cvtColor(yuvsrc, bgr, CV_YUV2BGR);
        std::vector<unsigned char> jpgbuf;
        cv::imencode(".jpg", bgr, jpgbuf);
        bgr.release();
        tcp_command cmd;
        cmd.type = IMAGE_DATA;
        cmd.size = jpgbuf.size();
        cmd.data.assign((char*)&(jpgbuf[0]), jpgbuf.size());
        server_->write(cmd);
    }
    cv::Mat frame_;
    std::shared_ptr<tcp_server> server_;
    int p_count_;
    std::string filename_;
};

#define VISION Vision::instance()

#endif