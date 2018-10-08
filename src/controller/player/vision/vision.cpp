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
    frame_.create(h_, w_, CV_8UC3);
    cudaSetDevice(0);
    cudaSetDeviceFlags(cudaDeviceMapHost);
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
        cv::Mat src;
        //frame_.copyTo(src);
        //src_gpu_mat_.download(frame_);
        //double start=clock();
        //cuda::cvtColor(src_gpu_mat_, dst_gpu_mat_, CV_YUV2BGR);
        //dst_gpu_mat_.download(src);
        cvtColor(frame_, src, CV_YUV2BGR);
        //double finish = clock();
        //std::cout<<"gpu: "<<(finish-start)/CLOCKS_PER_SEC<<std::endl;
        frame_mutex_.unlock();

        if(src.empty()) return;
/*
        //cudaBGRPacked2RGBPlanar(src.data, src_im_.data, w_, h_);
        int h = src.size().height;
        int w = src.size().width;
        int c = src.channels();
        image im = make_image(w, h, c);
        unsigned char *data = src.data;
        int step = w*c;
        int i, j, k;

        for(i = 0; i < h; ++i){
            for(k= 0; k < c; ++k){
                    for(j = 0; j < w; ++j){
                            im.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
                        }
                }
        }
        rgbgr_image(im);
        image sized = letterbox_image(src_im_, net_->w, net_->h);
        layer l = net_->layers[net_->n-1];
        float *X = sized.data;
        start=clock();
        network_predict(net_, X);
        int nboxes = 0;
        float nms=.45;
        detection *dets = get_network_boxes(net_, src_im_.w, src_im_.h, 0.5, 0.5, 0, 1, &nboxes);
        if (nms) do_nms_sort(dets, nboxes, l.classes, nms);

        for(int i=0;i<nboxes;i++)
        {
            rectangle(src, Point((dets[i].bbox.x-dets[i].bbox.w/2.0)*w_, (dets[i].bbox.y-dets[i].bbox.h/2.0)*h_),
                Point((dets[i].bbox.x+dets[i].bbox.w/2.0)*w_, (dets[i].bbox.y+dets[i].bbox.h/2.0)*h_), Scalar(255, 0, 0, 0));
        }

        free_detections(dets, nboxes);
        free_image(im);
        free_image(sized);*/
        if (OPTS->use_debug())
        {
            send_image(src);
        }
    }
}

void Vision::send_image(const cv::Mat &src)
{
    cv::Mat bgr;
    src.copyTo(bgr);
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
        //double start=clock();
        frame_ = cudaBuff2YUV(sptr->buffer(), sptr->buff_info());
        //Mat tmp = buff2mat(sptr->buffer(), sptr->buff_info());
        //cvtColor(tmp, frame_, CV_YUV2BGR);
        //yuyv2rgb(sptr->buffer()->start);
        //double finish = clock();
        //std::cout<<"cpu cvt: "<<(finish-start)/CLOCKS_PER_SEC<<std::endl;
        frame_mutex_.unlock();
    }
}

bool Vision::start(const sensor_ptr &s)
{
    server_ = std::dynamic_pointer_cast<tcp_server>(s);
    is_alive_ = true;
    start_timer();
    return true;
}

void Vision::stop()
{
    if (is_alive_)
    {
        delete_timer();
    }

    is_alive_ = false;
}


