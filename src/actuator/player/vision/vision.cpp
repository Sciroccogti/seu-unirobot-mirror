#include "vision.hpp"
#include "configuration.hpp"
#include "image/image_process.hpp"
#include <fstream>
#include <opencv2/dnn.hpp>
using namespace std;
using namespace image;

using namespace cv;

vision::vision(const sensor_ptr &s): timer(CONF->get_config_value<int>("vision_period"))
{
    server_ = std::dynamic_pointer_cast<tcp_server>(s);
    p_count_ = 0;
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
        frame_ = image_process::Buff2Mat_YUV(sptr->buffer(), sptr->buff_info());
        /*
        Mat bgr;
        cvtColor(frame_, bgr, CV_YUV2BGR);
        imwrite("test.jpg", bgr);
        */
        /*
        ofstream out;
        out.open("test.yuv", ios::app);
        out.write((char*)(frame_->start), frame_->size);
        out.close();
        */
        frame_mutex_.unlock();
    }
}

void vision::run()
{
    if(is_alive_)
    {
        p_count_ ++;
        frame_mutex_.lock();
        Mat bgr;
        cvtColor(frame_, bgr, CV_YUV2BGR);
        frame_mutex_.unlock();
        vector<unsigned char> jpgbuf;
        imencode(".jpg", bgr, jpgbuf);
        bgr.release();
        /*
        ofstream out;
        out.open("testt.jpg");
        out.write((char*)(&jpgbuf[0]), jpgbuf.size());
        out.close();
        */
        tcp_command cmd;
        cmd.type = IMAGE_DATA;
        if(server_!=nullptr)
        {
            cmd.size = jpgbuf.size();
            cmd.data.assign((char*)&(jpgbuf[0]), jpgbuf.size());
            server_->write(cmd);
        }
    }
}

vision::~vision()
{
    frame_.release();
}
