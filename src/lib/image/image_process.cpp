#include "image_process.hpp"

using namespace cv;

namespace imageproc
{
    bool cudaBuff2Image(const VideoBuffer *buff, const VideoBufferInfo &info, image &dst)
    {
        if(dst.data == nullptr) return false;
        int w = info.width, h = info.height;
        if(dst.h != h || dst.w != w) return false;
        unsigned char *dev_src;
        float *dev_dst;
        cudaError_t err;
        bool res = true;
        int src_size;
        int dst_size = h*w*3*sizeof(float);

        if(info.format == V4L2_PIX_FMT_YUYV)
            src_size = h*w*2*sizeof(unsigned char);
        else
            src_size = h*w*3*sizeof(unsigned char);

        err = cudaMalloc((void**)&dev_src, src_size);
        if(err != cudaSuccess)
        {
            res = false;
            goto cudaBUff2MatEnd;
        }
        err = cudaMalloc((void**)&dev_dst, dst_size);
        if(err != cudaSuccess)
        {
            res = false;
            goto cudaBUff2MatEnd;
        }
        err = cudaMemcpy(dev_src, buff->start, src_size, cudaMemcpyHostToDevice);
        if(err != cudaSuccess)
        {
            res = false;
            goto cudaBUff2MatEnd;
        }
        if(info.format == V4L2_PIX_FMT_YUYV)
            cudaYUYVPacked2YUVPlanar(dev_src, dev_dst, w, h);
        else if(info.format == V4L2_PIX_FMT_RGB24)
            cudaRGBPacked2RGBPlanar(dev_src, dev_dst, w, h);
        else if(info.format == V4L2_PIX_FMT_BGR24)
            cudaBGRPacked2RGBPlanar(dev_src, dev_dst, w, h);
        err = cudaMemcpy(dst.data, dev_dst, dst_size, cudaMemcpyDeviceToHost);
        if(err != cudaSuccess)
        {
            res = false;
            goto cudaBUff2MatEnd;
        }
        goto cudaBUff2MatEnd;

        cudaBUff2MatEnd:
        if(dev_src!=nullptr) cudaFree(dev_src);
        if(dev_dst!=nullptr) cudaFree(dev_dst);
        return res;
    }

    Mat buff2mat(const VideoBuffer *buf640x480, const VideoBufferInfo &info)
    {
        Mat dst640x480x3;

        if (info.width != 640 || info.height != 480)
        {
            return dst640x480x3;
        }

        dst640x480x3.create(info.height, info.width, CV_8UC3);

        unsigned char *src_ptr, *dst_ptr;
        unsigned short dst_offset, src_offset;
        unsigned short width = 640, height = 480;

        if (info.format == V4L2_PIX_FMT_YUYV)
        {
            for (unsigned short y = 0; y < height; y++)
            {
                src_ptr = (buf640x480->start + y * (width * 2));
                dst_ptr = dst640x480x3.data + y * (width * 3);

                for (unsigned short x = 0; x < width / 2; x++)
                {
                    dst_offset = x * 6;
                    src_offset = x * 4;
                    *(dst_ptr + dst_offset + 0) = *(src_ptr + src_offset + 0);
                    *(dst_ptr + dst_offset + 1) = *(src_ptr + src_offset + 1);
                    *(dst_ptr + dst_offset + 2) = *(src_ptr + src_offset + 3);
                    *(dst_ptr + dst_offset + 3) = *(src_ptr + src_offset + 2);
                    *(dst_ptr + dst_offset + 4) = *(src_ptr + src_offset + 1);
                    *(dst_ptr + dst_offset + 5) = *(src_ptr + src_offset + 3);
                }
            }
        }

        else if(info.format == V4L2_PIX_FMT_BGR24 || info.format == V4L2_PIX_FMT_RGB24)
        {
            memcpy(dst640x480x3.data, buf640x480->start, height*width*3);
        }

        return dst640x480x3;
    }
}

std::vector<cv::Mat> read_yuv(const std::string &filename, const int &w, const int &h)
{
    std::vector<Mat> res;
    std::ifstream in;
    int blksz = w * h * 3;
    in.open(filename.c_str());
    in.seekg(0, std::ios::end);
    int tsz = in.tellg();
    in.seekg(0);

    if (tsz % blksz != 0)
    {
        return res;
    }

    int frame_num = tsz / blksz;

    for (int i = 0; i < frame_num; i++)
    {
        Mat temp(h, w, CV_8UC3);
        in.read((char *)temp.data, blksz);
        res.push_back(temp);
    }

    return res;
}
