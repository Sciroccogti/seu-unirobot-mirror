#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <linux/videodev2.h>
#include "image_define.hpp"

void cudaYUYV2YUV(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h);
void cudaYUYV2BGR(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h);
void cudaYUYV2DST(unsigned char *in, unsigned char *bgr, float *rgb, unsigned char *color, const unsigned int &w, const unsigned int &h,
                  unsigned char color1, float c1hl, float c1hh, float c1sl, float c1sh, float c1il, float c1ih,
                  unsigned char color2, float c2hl, float c2hh, float c2sl, float c2sh, float c2il, float c2ih);

void cudaResize(float *in, int iw, int ih, float *sizedw, float *sized, int ow, int oh);

namespace imageproc
{
    cv::Mat cudaBuff2YUV(const VideoBuffer *buff, const VideoBufferInfo &info);
    cv::Mat cudaBuff2BGR(const VideoBuffer *buff, const VideoBufferInfo &info);
    cv::Mat buff2mat(const VideoBuffer *buf640x480, const VideoBufferInfo &info);

    inline cv::Vec3f BGR2HSI(unsigned char b, unsigned char g, unsigned char r)
    {
        float bn = b/255.0f;
        float gn = g/255.0f;
        float rn = r/255.0f;

        float min_v = std::min(bn, std::min(gn, rn));
        float H, S, I;
        float eps = 0.000001;
        I = (bn+gn+rn)/3.0f+eps;
        S = 1.0f-min_v/I;

        H = acos(0.5*(rn-gn+rn-bn)/sqrt((rn-gn)*(rn-gn)+(rn-bn)*(gn-bn)+eps));
        if(bn>gn) H = 2*M_PI-H;
        H = H*180.0f/M_PI;
        S = S*100.0f;
        I = I*100.0f;
        return cv::Vec3f(H,S,I);
    }
}

std::vector<cv::Mat> read_yuv(const std::string &filename, const int &w = 640, const int &h = 480);
inline void save_yuv(const cv::Mat &yuv, const std::string &filename, std::ios_base::openmode mode = std::ios_base::out | std::ios_base::trunc)
{
    std::ofstream out;
    out.open(filename.c_str(), mode);
    out.write((char *)(yuv.data), yuv.rows * yuv.cols * yuv.channels());
    out.close();
}