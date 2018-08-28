#include "vision.hpp"
#include "configuration.hpp"
#include "image/image_process.hpp"
#include <fstream>
#include <opencv2/dnn.hpp>
using namespace std;
using namespace image;

using namespace cv;
using namespace cv::cuda;
using namespace cv::dnn;

vision::vision(const sensor_ptr &s): timer(CONF.get_config_value<int>("vision_period"))
{
    server_ = dynamic_pointer_cast<tcp_server>(s);
    p_count_ = 0;
}

bool vision::start()
{
    cap_ = make_shared<capture>(shared_from_this());
    if(!cap_->start()) return false;
    width_ = cap_->buff_info().width;
    height_ = cap_->buff_info().height;
    frame_.create(height_, width_, CV_8UC3);
    is_alive_ = true;
    start_timer();
    return true;
}

void vision::stop()
{
    if(cap_!= nullptr)
    {
        cap_->detach(shared_from_this());
        cap_->stop();
    }
    if(is_alive_) delete_timer();
    is_alive_ = false;
}

void vision::updata(const pub_ptr& pub)
{
    if(pub == cap_)
    {
        frame_mutex_.lock();
        image_process::buffer2Mat_YUV(frame_, cap_->buffer(), cap_->buff_info());
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
