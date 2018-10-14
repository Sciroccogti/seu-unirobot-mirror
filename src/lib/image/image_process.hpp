#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <linux/videodev2.h>
#include "image_define.hpp"
#include <cuda_runtime.h>
#include "darknet/darknet.h"

void cudaYUYV2YUV(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h);
void cudaYUYV2BGR(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h);
void cudaYUYV2RGBPF(unsigned char *in, unsigned char *out, const unsigned int &w, const unsigned int &h);

namespace imageproc
{
    cv::Mat cudaBuff2YUV(const VideoBuffer *buff, const VideoBufferInfo &info);
    cv::Mat cudaBuff2BGR(const VideoBuffer *buff, const VideoBufferInfo &info);
    cv::Mat buff2mat(const VideoBuffer *buf640x480, const VideoBufferInfo &info);
}

std::vector<cv::Mat> read_yuv(const std::string &filename, const int &w = 640, const int &h = 480);
inline void save_yuv(const cv::Mat &yuv, const std::string &filename, std::ios_base::openmode mode = std::ios_base::out | std::ios_base::trunc)
{
    std::ofstream out;
    out.open(filename.c_str(), mode);
    out.write((char *)(yuv.data), yuv.rows * yuv.cols * yuv.channels());
    out.close();
}