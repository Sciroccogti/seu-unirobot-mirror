#include "vision.hpp"
#include "darknet/parser.h"
#include <cuda_runtime.h>

using namespace imageproc;
using namespace std;
using namespace cv;

Vision::Vision(): timer(CONF->get_config_value<int>("vision_period"))
{
    p_count_ = 0;
    filename_ = get_time() + ".yuv";
    w_ = CONF->get_config_value<int>("video.width");
    h_ = CONF->get_config_value<int>("video.height");
    src_size_ = w_*h_*2*sizeof(unsigned char);
    bgr_size_ = w_*h_*3*sizeof(unsigned char);
    rgbf_size_ = w_*h_*3*sizeof(float);

    yuyv_ = (unsigned char*)malloc(src_size_);
    cudaError_t err;
    err = cudaMalloc((void**)&dev_src_, src_size_);
    check_error(err);
    err = cudaMalloc((void**)&dev_bgr_, bgr_size_);
    check_error(err);
    err = cudaMalloc((void**)&dev_rgbf_, rgbf_size_);
    check_error(err);
}

Vision::~Vision()
{
    std::cout << "\033[32malgorithm:[Vision]   end!\033[0m\n";
    free(yuyv_);
    cudaFree(dev_src_);
    cudaFree(dev_bgr_);
    cudaFree(dev_rgbf_);
    cudaFree(dev_sized_);
}

void Vision::yuyv2dst()
{
    cudaError_t err;
    err = cudaMemcpy(dev_src_, yuyv_, src_size_, cudaMemcpyHostToDevice);
    check_error(err);
    cudaYUYV2DST(dev_src_, dev_bgr_, dev_rgbf_, w_, h_);
}

void Vision::run()
{
    if (is_alive_)
    {
        p_count_ ++;
        int h = h_;
        int w = w_;
        int c = 3;
        //image im = make_image(w, h, c);
        frame_mutex_.lock();
        yuyv2dst();
        Mat bgr(h_, w_, CV_8UC3);
        cudaError_t err;
        err = cudaMemcpy(bgr.data, dev_bgr_, bgr_size_, cudaMemcpyDeviceToHost);
        check_error(err);
        //memcpy(bgr.data, dev_bgr_, bgr_size_);
        //memcpy(im.data, dev_rgbf_, rgbf_size_);
        //cudaResize(dev_rgbf_, w_, h_, dev_sized_, net_.w, net_.h);
        frame_mutex_.unlock();
        //if(bgr.empty()) return;

        //image sized = resize_image(im, net_.w, net_.h);

        layer l = net_.layers[net_.n-1];
        //float *X = sized.data;
        double t1 = clock();
        network_predict(net_, dev_sized_, 1);
        int nboxes = 0;
        float nms=.45;
        detection *dets = get_network_boxes(&net_, w_, h_, 0.5, 0.5, 0, 1, &nboxes, 0);
        if (nms) do_nms_sort(dets, nboxes, l.classes, nms);
        cout<<(clock()-t1)/CLOCKS_PER_SEC<<endl;
        for(int i=0;i<nboxes;i++)
        {
            rectangle(bgr, Point((dets[i].bbox.x-dets[i].bbox.w/2.0)*w, (dets[i].bbox.y-dets[i].bbox.h/2.0)*h),
                Point((dets[i].bbox.x+dets[i].bbox.w/2.0)*w, (dets[i].bbox.y+dets[i].bbox.h/2.0)*h), Scalar(255, 0, 0, 0));
            for(int j=0;j<l.classes;j++)
            {
                if(dets[i].prob[j]>0.2)
                {
                    putText(bgr, names_[j], Point(dets[i].bbox.x*w, dets[i].bbox.y*h), FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),1,8);
                }
            }
        }
        free_detections(dets, nboxes);
        //free_image(im);
        //free_image(sized);

        if (OPTS->use_debug())
        {
            send_image(bgr);
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
        memcpy(yuyv_, sptr->buffer()->start, src_size_);
        frame_mutex_.unlock();
    }
}

bool Vision::start(const sensor_ptr &s)
{
    server_ = std::dynamic_pointer_cast<tcp_server>(s);
    names_.clear();
    ifstream ifs(CONF->get_config_value<string>("net_names_file"));
    while(!ifs.eof())
    {
        string s;
        ifs>>s;
        names_.push_back(s);
    }
    ifs.close();
    net_.gpu_index = 0;
    net_ = parse_network_cfg_custom((char*)CONF->get_config_value<string>("net_cfg_file").c_str(), 1);
    load_weights(&net_, (char*)CONF->get_config_value<string>("net_weights_file").c_str());
    set_batch_network(&net_, 1);
    fuse_conv_batchnorm(net_);
    calculate_binary_weights(net_);
    srand(2222222);
    sized_size_ = net_.w*net_.h*3* sizeof(float);
    cudaError_t err;
    err = cudaMallocManaged((void**)&dev_sized_, sized_size_);
    check_error(err);
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
    free_network(net_);
    is_alive_ = false;
}


