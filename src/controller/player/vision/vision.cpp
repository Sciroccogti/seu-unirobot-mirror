#include "vision.hpp"
#include "darknet/parser.h"
#include "parser/color_parser.hpp"
#include <cuda_runtime.h>

using namespace imageproc;
using namespace std;
using namespace cv;

Vision::Vision(): timer(CONF->get_config_value<int>("vision_period"))
{
    p_count_ = 0;
    is_busy_ = false;
    filename_ = get_time() + ".yuv";
    w_ = CONF->get_config_value<int>("video.width");
    h_ = CONF->get_config_value<int>("video.height");
    parser::color_parser::parse(CONF->get_config_value<string>(CONF->player()+".color_file"), colors_);
}

Vision::~Vision()
{
    std::cout << "\033[32malgorithm:[Vision]   end!\033[0m\n";
}

void Vision::yuyv2dst()
{
    cudaError_t err;
    err = cudaMemcpy(dev_src_, yuyv_, src_size_, cudaMemcpyHostToDevice);
    check_error(err);
    cudaYUYV2DST(dev_src_, dev_bgr_, dev_rgbf_, dev_hsi_, w_, h_);
}

void Vision::run()
{
    if (is_alive_)
    {
        p_count_ ++;
        if(is_busy_) return;
        frame_mutex_.lock();
        yuyv2dst();
        frame_mutex_.unlock();
        cudaError_t err;
        /*
        cudaResize(dev_rgbf_, w_, h_, dev_wsized_, dev_sized_, net_.w, net_.h);
        is_busy_ = true;
        layer l = net_.layers[net_.n-1];
        //double t1 = clock();
        network_predict(net_, dev_sized_, 1);
        int nboxes = 0;
        float nms=.45;
        detection *dets = get_network_boxes(&net_, w_, h_, 0.5, 0.5, 0, 1, &nboxes, 0);
        if (nms) do_nms_sort(dets, nboxes, l.classes, nms);
        //cout<<(clock()-t1)/CLOCKS_PER_SEC<<endl;
        */
        if (OPTS->use_debug())
        {
            Mat bgr(h_, w_, CV_8UC3);
            err = cudaMemcpy(bgr.data, dev_bgr_, bgr_size_, cudaMemcpyDeviceToHost);
            check_error(err);
            float *hsi;
            hsi = (float*)malloc(hsi_size_);
            err = cudaMemcpy(hsi, dev_hsi_, hsi_size_, cudaMemcpyDeviceToHost);
            int i,j;
            check_error(err);
            int planesize = w_*h_;
            for(j=0;j<h_;j++)
            {
                for(i=0;i<w_;i++)
                {
                    if(hsi[j*w_+i] >= colors_[COLOR_GREEN].H.minimum&&hsi[j*w_+i] <= colors_[COLOR_GREEN].H.maximum
                    &&hsi[planesize+j*w_+i] >= colors_[COLOR_GREEN].S.minimum&&hsi[planesize+j*w_+i] <= colors_[COLOR_GREEN].S.maximum
                    &&hsi[2*planesize+j*w_+i] >= colors_[COLOR_GREEN].I.minimum&&hsi[2*planesize+j*w_+i] <= colors_[COLOR_GREEN].I.maximum)
                    {
                        bgr.data[j*w_*3+i*3] = 0;
                        bgr.data[j*w_*3+i*3+1] = 255;
                        bgr.data[j*w_*3+i*3+2] = 0;
                    }
                    else if(hsi[j*w_+i] >= colors_[COLOR_WHITE].H.minimum&&hsi[j*w_+i] <= colors_[COLOR_WHITE].H.maximum
                              &&hsi[planesize+j*w_+i] >= colors_[COLOR_WHITE].S.minimum&&hsi[planesize+j*w_+i] <= colors_[COLOR_WHITE].S.maximum
                              &&hsi[2*planesize+j*w_+i] >= colors_[COLOR_WHITE].I.minimum&&hsi[2*planesize+j*w_+i] <= colors_[COLOR_WHITE].I.maximum)
                    {
                        bgr.data[j*w_*3+i*3] = 255;
                        bgr.data[j*w_*3+i*3+1] = 255;
                        bgr.data[j*w_*3+i*3+2] = 255;
                    }
                    else
                    {
                        bgr.data[j*w_*3+i*3] = 0;
                        bgr.data[j*w_*3+i*3+1] = 0;
                        bgr.data[j*w_*3+i*3+2] = 0;
                    }
                }
            }
            free(hsi);
            /*
            for(int i=0;i<nboxes;i++)
            {
                rectangle(bgr, Point((dets[i].bbox.x-dets[i].bbox.w/2.0)*w_, (dets[i].bbox.y-dets[i].bbox.h/2.0)*h_),
                          Point((dets[i].bbox.x+dets[i].bbox.w/2.0)*w_, (dets[i].bbox.y+dets[i].bbox.h/2.0)*h_),
                          Scalar(255, 0, 0, 0));
                for(int j=0;j<l.classes;j++)
                {
                    if(dets[i].prob[j]>0.2)
                    {
                        putText(bgr, names_[j], Point(dets[i].bbox.x*w_, dets[i].bbox.y*h_),
                                FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),1,8);
                    }
                }
            }
            */
            send_image(bgr);
        }
        //free_detections(dets, nboxes);
        is_busy_ = false;
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
    //load_weights(&net_, (char*)CONF->get_config_value<string>("net_weights_file").c_str());
    //set_batch_network(&net_, 1);
    //fuse_conv_batchnorm(net_);
    //calculate_binary_weights(net_);
    srand(2222222);
    src_size_ = w_*h_*2*sizeof(unsigned char);
    bgr_size_ = w_*h_*3*sizeof(unsigned char);
    rgbf_size_ = w_*h_*3*sizeof(float);
    hsi_size_ = w_*h_*3*sizeof(float);
    sized_size_ = net_.w*net_.h*3*sizeof(float);
    sizew_size_ = net_.w*h_*3*sizeof(float);
    yuyv_ = (unsigned char*)malloc(src_size_);
    cudaError_t err;
    err = cudaMalloc((void**)&dev_src_, src_size_);
    check_error(err);
    err = cudaMalloc((void**)&dev_bgr_, bgr_size_);
    check_error(err);
    err = cudaMalloc((void**)&dev_rgbf_, rgbf_size_);
    check_error(err);
    err = cudaMalloc((void**)&dev_hsi_, hsi_size_);
    err = cudaMallocManaged((void**)&dev_wsized_, sizew_size_);
    check_error(err);
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
        free_network(net_);
        free(yuyv_);
        cudaFree(dev_src_);
        cudaFree(dev_bgr_);
        cudaFree(dev_rgbf_);
        cudaFree(dev_hsi_);
        cudaFree(dev_sized_);
        cudaFree(dev_wsized_);
    }
    is_alive_ = false;
}

void Vision::caculateColor(imageproc::Color c, int x, int y, int w, int h)
{
    if(is_busy_) return;
    is_busy_ = true;
    if(c == COLOR_CLEAN)
    {
        for(auto &cl: colors_)
        {
            cl.second.H.minimum = 0;
            cl.second.H.maximum = 0;
            cl.second.S.minimum = 0;
            cl.second.S.maximum = 0;
            cl.second.I.minimum = 0;
            cl.second.I.maximum = 0;
        }
    }
    else
    {
        float *hsi;
        hsi = (float*)malloc(hsi_size_);
        cudaError_t err;
        err = cudaMemcpy(hsi, dev_hsi_, hsi_size_, cudaMemcpyDeviceToHost);
        check_error(err);
        int planesize = w_*h_;
        int n = w*h;
        float sumh = 0.0, sums=0.0, sumi = 0.0;
        int i,j;
        for(j=y;j<y+h;j++)
        {
            for(i=x;i<x+w;i++)
            {
                sumh += hsi[j*w_+i];
                sums += hsi[planesize+j*w_+i];
                sumi += hsi[2*planesize+j*w_+i];
            }
        }
        float mh = sumh/n;
        float ms = sums/n;
        float mi = sumi/n;
        float sh=0.0, ss=0.0, si=0.0;
        for(j=y;j<y+h;j++)
        {
            for(i=x;i<x+w;i++)
            {
                sh += pow(hsi[j*w_+i]-mh, 2);
                ss += pow(hsi[planesize+j*w_+i]-ms, 2);
                si += pow(hsi[2*planesize+j*w_+i]-mi, 2);
            }
        }
        sh = sqrt(sh/n);
        ss = sqrt(ss/n);
        si = sqrt(si/n);
        if(colors_[c].H.minimum == 0 && colors_[c].H.maximum==0)
        {
            colors_[c].H.minimum = mh-3.0f*sh;
            colors_[c].H.maximum = mh+3.0f*sh;
            colors_[c].S.minimum = ms-3.0f*ss;
            colors_[c].S.maximum = ms+3.0f*ss;
            colors_[c].I.minimum = mi-3.0f*si;
            colors_[c].I.maximum = mi+3.0f*si;
        }
        else
        {
            colors_[c].H.minimum = (mh-3.0f*sh)<colors_[c].H.minimum?(mh-3.0f*sh):colors_[c].H.minimum;
            colors_[c].H.maximum = (mh+3.0f*sh)>colors_[c].H.maximum?(mh+3.0f*sh):colors_[c].H.maximum;
            colors_[c].S.minimum = (ms-3.0f*ss)<colors_[c].S.minimum?(ms-3.0f*ss):colors_[c].S.minimum;
            colors_[c].S.maximum = (ms+3.0f*ss)>colors_[c].S.maximum?(ms+3.0f*ss):colors_[c].S.maximum;
            colors_[c].I.minimum = (mi-3.0f*si)<colors_[c].I.minimum?(mi-3.0f*si):colors_[c].I.minimum;
            colors_[c].I.maximum = (mi+3.0f*si)>colors_[c].I.maximum?(mi+3.0f*si):colors_[c].I.maximum;
        }
    }
    parser::color_parser::save(CONF->get_config_value<string>(CONF->player()+".color_file"), colors_);
    is_busy_ = false;
}
