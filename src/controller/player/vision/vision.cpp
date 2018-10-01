#include "vision.hpp"

using namespace imageproc;
using namespace std;

Vision::Vision(): timer(CONF->get_config_value<int>("vision_period"))
{
    p_count_ = 0;
    filename_ = get_time() + ".yuv";
    gpu_index = 0;
    cuda_set_device(gpu_index);
}

Vision::~Vision()
{
    std::cout << "\033[32malgorithm:[Vision]   end!\033[0m\n";
}

void Vision::run()
{
    if (is_alive_)
    {
        p_count_ ++;
        frame_mutex_.lock();
        cv::Mat yuv(frame_);
        frame_mutex_.unlock();

        if (frame_.empty())
        {
            return;
        }

        if (OPTS->image_record())
        {
            image_process::save_yuv(yuv, filename_, ios::app);
        }

        if (OPTS->use_debug())
        {
            send_image(yuv);
        }
    }
}

void Vision::send_image(const cv::Mat &yuvsrc)
{
    cv::Mat bgr;
    cvtColor(yuvsrc, bgr, CV_YUV2BGR);
    std::vector<unsigned char> jpgbuf;
    cv::imencode(".jpg", bgr, jpgbuf);
    bgr.release();
    tcp_command cmd;
    cmd.type = IMG_DATA;
    cmd.size = jpgbuf.size();
    cmd.data.assign((char *) & (jpgbuf[0]), jpgbuf.size());
    server_->write(cmd);
}

void Vision::updata(const pro_ptr &pub, const int &type)
{
    shared_ptr<camera> sptr = dynamic_pointer_cast<camera>(pub);

    if (type == sensor::SENSOR_CAMERA)
    {
        frame_mutex_.lock();
        frame_ = imageproc::image_process::buff2yuv_mat(sptr->buffer(), sptr->buff_info());
        frame_mutex_.unlock();
    }
}

bool Vision::start(const sensor_ptr &s)
{
    server_ = std::dynamic_pointer_cast<tcp_server>(s);
    is_alive_ = true;
    net_ = load_network((char *)CONF->get_config_value<string>("net_cfg_file").c_str(),
                        (char *)CONF->get_config_value<string>("net_weights_file").c_str(), 0);
    set_batch_network(net_, 1);
    srand(2222222);
    start_timer();
    return true;
}

void Vision::stop()
{
    free_network(net_);

    if (is_alive_)
    {
        delete_timer();
    }

    is_alive_ = false;
}


