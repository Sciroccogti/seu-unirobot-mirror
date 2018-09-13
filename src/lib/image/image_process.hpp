#ifndef SEU_UNIROBOT_IMAGE_PROCESS_HPP
#define SEU_UNIROBOT_IMAGE_PROCESS_HPP

#include <memory>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <linux/videodev2.h>
#include "image_define.hpp"

namespace image
{
    class image_process
    {
    public:
        static cv::Mat buff2yuv_mat(const VideoBuffer *buf640x480, const VideoBufferInfo &info)
        {
            cv::Mat dst640x480x3;
            if(info.width!=640 || info.height != 480) return dst640x480x3;
            dst640x480x3.create(info.height, info.width, CV_8UC3);
            
            unsigned char *src_ptr, *dst_ptr;
            unsigned short dst_offset, src_offset;
            unsigned short width = 640, heigth = 480;
            
            if(info.format == V4L2_PIX_FMT_YUYV)
            {
                for(unsigned short y=0; y<info.height; y++)
                {
                    src_ptr = (buf640x480->start + y*(width*2));
                    dst_ptr = dst640x480x3.data+y*(width*3);
                    for(unsigned short x=0; x<width/2; x++)
                    {
                        dst_offset = x*6;
                        src_offset = x*4;
                        *(dst_ptr+dst_offset+0) = *(src_ptr+src_offset+0);
                        *(dst_ptr+dst_offset+1) = *(src_ptr+src_offset+1);
                        *(dst_ptr+dst_offset+2) = *(src_ptr+src_offset+3);
                        *(dst_ptr+dst_offset+3) = *(src_ptr+src_offset+2);
                        *(dst_ptr+dst_offset+4) = *(src_ptr+src_offset+1);
                        *(dst_ptr+dst_offset+5) = *(src_ptr+src_offset+3);
                    }
                }
            }
            return dst640x480x3;
        }
        
        static void save_yuv(const cv::Mat &yuv,const std::string &filename, std::ios_base::openmode mode=std::ios_base::out | std::ios_base::trunc)
        {
            std::ofstream out;
            out.open(filename.c_str(), mode);
            out.write((char*)(yuv.data), yuv.rows*yuv.cols*yuv.channels());
            out.close();
        }
        
        static std::vector<cv::Mat> read_yuv(const std::string &filename, const int w=640, const int h=480)
        {
            std::vector<cv::Mat> res;
            std::ifstream in;
            int blksz=w*h*3;
            in.open(filename.c_str());
            in.seekg(0, std::ios::end);
            int tsz = in.tellg();
            in.seekg(0);
            if(tsz%blksz != 0) return res;
            int frame_num = tsz/blksz;
            for(int i=0;i<frame_num;i++)
            {
                cv::Mat temp(h,w,CV_8UC3);
                in.read((char*)temp.data, blksz);
                res.push_back(temp);
            }
            return res;
        }
    };
}

#endif