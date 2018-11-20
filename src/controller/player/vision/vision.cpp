#include "vision.hpp"
#include "darknet/parser.h"
#include "parser/color_parser.hpp"
#include <cuda_runtime.h>
#include "image/cuda_process.h"

using namespace imageproc;
using namespace std;
using namespace cv;

Vision::Vision(): timer(CONF->get_config_value<int>("vision_period"))
{
    p_count_ = 0;
    is_busy_ = false;
    w_ = CONF->get_config_value<int>("image.width");
    h_ = CONF->get_config_value<int>("image.height");
    img_sd_type_ = IMAGE_SEND_RESULT;
    camera_src_ = nullptr;
    use_mv_ = CONF->get_config_value<bool>("video.use_mv");
}

Vision::~Vision()
{
    std::cout << "\033[32malgorithm:[Vision]   end!\033[0m\n";
}

void Vision::src2dst()
{
    cudaError_t err;
    err = cudaMemcpy(dev_src_, camera_src_, src_size_, cudaMemcpyHostToDevice);
    check_error(err);
    if (use_mv_)
        cudaBayer2BGR(dev_src_, dev_bgr_,  camera_w_,  camera_h_,  0.0,  1.0, 1.0, 1.0);
    else
        cudaYUYV2BGR(dev_src_,  dev_bgr_,  camera_w_,  camera_h_);
    cudaResizePacked(dev_bgr_, camera_w_,  camera_h_,  dev_sized_bgr_, w_,  h_);
    cudaBGR2RGBfp(dev_sized_bgr_, dev_rgbfp_, w_, h_);
}

void Vision::run()
{
    if (is_alive_)
    {
        
        p_count_ ++;
        if(is_busy_) return;
        frame_mutex_.lock();
        double t1 = clock();
        src2dst();
        double t2 = clock();
        cout << (t2-t1)/CLOCKS_PER_SEC << endl;
        frame_mutex_.unlock();
        cudaError_t err;
        Mat bgr(h_, w_, CV_8UC3);
        err = cudaMemcpy(bgr.data, dev_bgr_, bgr_size_, cudaMemcpyDeviceToHost);
        check_error(err);
        send_image(bgr);
        
        /*
        cudaResizePlanar(dev_rgbfp_, w_, h_, dev_sized_, net_.w, net_.h);
        is_busy_ = true;
        layer l = net_.layers[net_.n-1];
        //double t1 = clock();
        network_predict(net_, dev_sized_, 1);
        int nboxes = 0;
        float nms=.45;
        detection *dets = get_network_boxes(&net_, w_, h_, 0.5, 0.5, 0, 1, &nboxes, 0);
        if (nms) do_nms_sort(dets, nboxes, l.classes, nms);
        //cout<<(clock()-t1)/CLOCKS_PER_SEC<<endl;

        if (OPTS->use_debug())
        {
            Mat bgr(h_, w_, CV_8UC3);
            err = cudaMemcpy(bgr.data, dev_bgr_, bgr_size_, cudaMemcpyDeviceToHost);
            check_error(err);
            if(img_sd_type_ == IMAGE_SEND_ORIGIN)
                send_image(bgr);
            else
            {
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

                send_image(bgr);
            }
        }
        free_detections(dets, nboxes);
        is_busy_ = false;
        */
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
    
    if(!is_alive_) return;
    shared_ptr<camera> sptr = dynamic_pointer_cast<camera>(pub);
    
    if (camera_src_ ==  nullptr)
    {
        camera_w_ = sptr->camera_w();
        camera_h_ = sptr->camera_h();
        camera_size_ = sptr->camera_size();
        src_size_ = camera_size_;
        bgr_size_ = camera_w_*camera_h_*3;
        camera_src_ = (unsigned char*)malloc(camera_size_);
        cudaError_t err;
        err = cudaMalloc((void**) &dev_src_,  src_size_);
        check_error(err);
        err = cudaMalloc((void**) &dev_bgr_, bgr_size_);
        check_error(err);
    }
    
    if (type == sensor::SENSOR_CAMERA)
    {
        frame_mutex_.lock();
        memcpy(camera_src_, sptr->buffer(), src_size_);
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

    sized_bgr_size_ = w_*h_*3;
    rgbf_size_ = w_*h_*3*sizeof(float);
    sized_size_ = net_.w*net_.h*3*sizeof(float);

    cudaError_t err;
    err = cudaMalloc((void**)&dev_sized_bgr_, sized_bgr_size_);
    check_error(err);
    err = cudaMalloc((void**)&dev_rgbfp_, rgbf_size_);
    check_error(err);
    err = cudaMalloc((void**)&dev_sized_, sized_size_);
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
        free(camera_src_);
        cudaFree(dev_sized_bgr_);
        cudaFree(dev_src_);
        cudaFree(dev_bgr_);
        cudaFree(dev_rgbfp_);
        cudaFree(dev_sized_);
    }
    is_alive_ = false;
}

void Vision::caculateColor(imageproc::ColorType c, int x, int y, int w, int h)
{
    if(is_busy_) return;
    is_busy_ = true;
    if(c == COLOR_CLEAN)
    {
        for(auto &cl: colors_)
        {
            cl.H.minimum = 0;
            cl.H.maximum = 0;
            cl.S.minimum = 0;
            cl.S.maximum = 0;
            cl.I.minimum = 0;
            cl.I.maximum = 0;
        }
    }
    else
    {
        float *hsi;
        hsi = (float*)malloc(w*h*3*sizeof(float));
        Mat bgr(h_, w_, CV_8UC3);
        cudaError_t err;
        err = cudaMemcpy(bgr.data, dev_bgr_, bgr_size_, cudaMemcpyDeviceToHost);
        check_error(err);
        int n = w*h;
        float sumh = 0.0, sums=0.0, sumi = 0.0;
        int i,j;
        for(j=y;j<y+h;j++)
        {
            for(i=x;i<x+w;i++)
            {
                Vec3f tmp = BGR2HSI(bgr.data[j*w_*3+i*3], bgr.data[j*w_*3+i*3+1], bgr.data[j*w_*3+i*3+2]);
                hsi[(j-y)*w+i-x] = tmp[0];
                hsi[n+(j-y)*w+i-x] = tmp[1];
                hsi[2*n+(j-y)*w+i-x] = tmp[2];
            }
        }

        for(j=0;j<h;j++)
        {
            for(i=0;i<w;i++)
            {
                sumh += hsi[j*w+i];
                sums += hsi[n+j*w+i];
                sumi += hsi[2*n+j*w+i];
            }
        }
        float mh = sumh/n;
        float ms = sums/n;
        float mi = sumi/n;
        float sh=0.0, ss=0.0, si=0.0;
        for(j=0;j<h;j++)
        {
            for(i=0;i<w;i++)
            {
                sh += pow(hsi[j*w+i]-mh, 2);
                ss += pow(hsi[n+j*w+i]-ms, 2);
                si += pow(hsi[2*n+j*w+i]-mi, 2);
            }
        }
        sh = sqrt(sh/n);
        ss = sqrt(ss/n);
        si = sqrt(si/n);
        for(i=0;i<colors_.size();i++)
        {
            if(colors_[i].c == c) break;
        }
        if(i == colors_.size()) return;

        if(colors_[i].H.minimum == 0 && colors_[i].H.maximum==0)
        {
            colors_[i].H.minimum = mh-3.0f*sh;
            colors_[i].H.maximum = mh+3.0f*sh;
            colors_[i].S.minimum = ms-3.0f*ss;
            colors_[i].S.maximum = ms+3.0f*ss;
            colors_[i].I.minimum = mi-3.0f*si;
            colors_[i].I.maximum = mi+3.0f*si;
        }
        else
        {
            colors_[i].H.minimum = (mh-3.0f*sh)<colors_[i].H.minimum?(mh-3.0f*sh):colors_[i].H.minimum;
            colors_[i].H.maximum = (mh+3.0f*sh)>colors_[i].H.maximum?(mh+3.0f*sh):colors_[i].H.maximum;
            colors_[i].S.minimum = (ms-3.0f*ss)<colors_[i].S.minimum?(ms-3.0f*ss):colors_[i].S.minimum;
            colors_[i].S.maximum = (ms+3.0f*ss)>colors_[i].S.maximum?(ms+3.0f*ss):colors_[i].S.maximum;
            colors_[i].I.minimum = (mi-3.0f*si)<colors_[i].I.minimum?(mi-3.0f*si):colors_[i].I.minimum;
            colors_[i].I.maximum = (mi+3.0f*si)>colors_[i].I.maximum?(mi+3.0f*si):colors_[i].I.maximum;
        }
        free(hsi);
    }
    parser::color_parser::save(CONF->get_config_value<string>(CONF->player()+".color_file"), colors_);
    is_busy_ = false;
}
