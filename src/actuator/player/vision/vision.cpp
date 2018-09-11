#include "vision.hpp"
#include "configuration.hpp"
#include "image/image_process.hpp"
#include <fstream>
#include "options/options.hpp"
#include "common.hpp"

using namespace std;
using namespace image;

using namespace cv;

vision::vision(const sensor_ptr &s): timer(CONF->get_config_value<int>("vision_period"))
{
    server_ = std::dynamic_pointer_cast<tcp_server>(s);
    p_count_ = 0;
    filename_ = get_time()+".yuv";
}

bool vision::start()
{
    is_alive_ = true;
    start_timer();
    return true;
}

void vision::stop()
{
    if(is_alive_) delete_timer();
    is_alive_ = false;
}

void vision::updata(const pro_ptr& pub, const int &type)
{
    std::shared_ptr<camera> sptr = std::dynamic_pointer_cast<camera>(pub);
    if(type == sensor::SENSOR_CAMERA)
    {
        frame_mutex_.lock();
        frame_ = image_process::buff2yuv_mat(sptr->buffer(), sptr->buff_info());
        frame_mutex_.unlock();
    }
}

void vision::run()
{
    if(is_alive_)
    {
        p_count_ ++;
        frame_mutex_.lock();
        Mat yuv(frame_);
        frame_mutex_.unlock();
        if(OPTS->image_record())
            image_process::save_yuv(yuv, filename_, ios::app);
        if(OPTS->use_debug())
            send_image(yuv);
    }
}

void vision::send_image(const cv::Mat &yuvsrc)
{
    Mat bgr;
    cvtColor(yuvsrc, bgr, CV_YUV2BGR);
    vector<unsigned char> jpgbuf;
    imencode(".jpg", bgr, jpgbuf);
    bgr.release();
    tcp_command cmd;
    cmd.type = IMAGE_DATA;
    cmd.size = jpgbuf.size();
    cmd.data.assign((char*)&(jpgbuf[0]), jpgbuf.size());
    server_->write(cmd);
}

vision::~vision()
{
    frame_.release();
}
