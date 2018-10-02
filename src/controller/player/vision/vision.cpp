#include "vision.hpp"

using namespace imageproc;
using namespace std;
using namespace cv;

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
        clock_t start,finish;
        double totaltime;
        
        frame_mutex_.lock();
        cv::Mat src;
        cvtColor(frame_, src, CV_YUV2BGR);
        frame_mutex_.unlock();

        if (frame_.empty())
        {
            return;
        }

        if (OPTS->image_record())
        {
            image_process::save_yuv(src, filename_, ios::app);
        }
        
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
        start=clock();
        image sized = letterbox_image(im, net_->w, net_->h);
        layer l = net_->layers[net_->n-1];
        float *X = sized.data;
        network_predict(net_, X);
        int nboxes = 0;
        float nms=.45;
        detection *dets = get_network_boxes(net_, im.w, im.h, 0.5, 0.5, 0, 1, &nboxes);
        if (nms) do_nms_sort(dets, nboxes, l.classes, nms);
        /*
        for(int i=0;i<nboxes;i++)
        {
            rectangle(src, Point((dets[i].bbox.x-dets[i].bbox.w/2.0)*w, (dets[i].bbox.y-dets[i].bbox.h/2.0)*h), 
                Point((dets[i].bbox.x+dets[i].bbox.w/2.0)*w, (dets[i].bbox.y+dets[i].bbox.h/2.0)*h), Scalar(255, 0, 0, 0));
        }
        */
        free_detections(dets, nboxes);
        free_image(im);
        free_image(sized);
        finish = clock();
        totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
        cout<<"use time: "<<totaltime<<endl;
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
        frame_ = imageproc::image_process::buff2mat(sptr->buffer(), sptr->buff_info());
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


