#include "vision.hpp"

using namespace imageproc;
using namespace std;
using namespace cv;

Vision::Vision(): timer(CONF->get_config_value<int>("vision_period"))
{
    p_count_ = 0;
    filename_ = get_time() + ".yuv";
    w_ = CONF->get_config_value<int>("video.width");
    h_ = CONF->get_config_value<int>("video.height");
    src_im_ = make_image(w_, h_, 3);
    frame_.create(h_, w_, CV_8UC3);
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
        clock_t start,finish;
        double totaltime;
        
        frame_mutex_.lock();
        cv::Mat src;
        frame_.copyTo(src);
        frame_mutex_.unlock();

        if(src.empty()) return;
        cudaBGRPacked2RGBPlanar(src.data, src_im_.data, w_, h_);

        image sized = letterbox_image(src_im_, net_->w, net_->h);
        layer l = net_->layers[net_->n-1];
        float *X = sized.data;
        start=clock();
        network_predict(net_, X);
        int nboxes = 0;
        float nms=.45;
        detection *dets = get_network_boxes(net_, src_im_.w, src_im_.h, 0.5, 0.5, 0, 1, &nboxes);
        finish = clock();
        totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
        cout<<"use time: "<<totaltime<<endl;
        if (nms) do_nms_sort(dets, nboxes, l.classes, nms);

        for(int i=0;i<nboxes;i++)
        {
            rectangle(src, Point((dets[i].bbox.x-dets[i].bbox.w/2.0)*w_, (dets[i].bbox.y-dets[i].bbox.h/2.0)*h_),
                Point((dets[i].bbox.x+dets[i].bbox.w/2.0)*w_, (dets[i].bbox.y+dets[i].bbox.h/2.0)*h_), Scalar(255, 0, 0, 0));
        }

        free_detections(dets, nboxes);
        free_image(sized);
        if (OPTS->use_debug())
        {
            send_image(src, V4L2_PIX_FMT_BGR24);
        }
    }
}

void Vision::send_image(const cv::Mat &src, const unsigned int &fmt)
{
    cv::Mat bgr;
    if(fmt == V4L2_PIX_FMT_YUV444)
        cvtColor(src, bgr, CV_YUV2BGR);
    else if(fmt == V4L2_PIX_FMT_BGR24)
        src.copyTo(bgr);
    else return;
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
        if(sptr->buff_info().format == V4L2_PIX_FMT_BGR24 || sptr->buff_info().format == V4L2_PIX_FMT_RGB24)
            memcpy(frame_.data, sptr->buffer()->start, w_*h_*3);
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


