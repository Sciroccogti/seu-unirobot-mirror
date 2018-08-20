#include "vision.hpp"
#include "configuration.hpp"
#include "image/image_process.hpp"
#include <fstream>

using namespace std;
using namespace image;
using namespace cv;

vision::vision(const sensor_ptr &s): timer(CONF.get_config_value<int>("vision_period"))
{
    //frame_ = make_shared<VideoFrame>(640, 480, 3);
    frame_.create(height, width, CV_8UC3);
    server_ = dynamic_pointer_cast<server>(s);
    p_count_ = 0;
}

bool vision::start()
{
    cap_ = make_shared<capture>(shared_from_this());
    if(!cap_->start()) return false;
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
        if(server_!=nullptr)
        {
            LOG(LOG_INFO, "send: "<<p_count_);
            server_->write(comm::tcp_packet::IMAGE_DATA, jpgbuf.size(), (char*)&(jpgbuf[0]));
        }
        //LOG(LOG_INFO, "origin size: "<<bgr.rows*bgr.cols*bgr.channels()<<"\tencode size: "<<jpgbuf.size()<<"\tpencentage: "<<(float)jpgbuf.size()/(float)(bgr.rows*bgr.cols*bgr.channels()));
    }
}

vision::~vision()
{

}
