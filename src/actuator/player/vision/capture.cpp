#include <libv4l2.h>
#include <sys/mman.h>
#include <errno.h>
#include <functional>
#include "capture.hpp"
#include "configuration.hpp"
#include "class_exception.hpp"

using namespace std;
using namespace image;

capture::capture(const sub_ptr& s): sensor("capture")
{
    attach(s);
    format_map_["V4L2_PIX_FMT_YUYV"] = V4L2_PIX_FMT_YUYV;
    format_map_["V4L2_PIX_FMT_MJPEG"] = V4L2_PIX_FMT_MJPEG;
    format_map_["V4L2_PIX_FMT_JPEG"] = V4L2_PIX_FMT_JPEG;
    cfg_.dev_name = CONF.get_config_value<string>("hardware.capture.dev_name");
    cfg_.buff_num = CONF.get_config_value<int>("hardware.capture.buff_num");
    cfg_.format = CONF.get_config_value<string>("hardware.capture.format");
    cfg_.height = CONF.get_config_value<int>("hardware.capture.height");
    cfg_.width = CONF.get_config_value<int>("hardware.capture.width");
}

bool capture::start()
{
    if(!open()) return false;
    if(!init()) return false;
    td_ = thread(bind(&capture::run, this));
    return true;
}

void capture::run()
{
    is_alive_ = true;
    buf_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf_.memory = V4L2_MEMORY_MMAP;

    while(is_alive_)
    {
        if (v4l2_ioctl(fd_, VIDIOC_DQBUF, &buf_) == -1)
        {
            LOG(LOG_ERROR, "VIDIOC_DQBUF failed.");
            break;
        }
        num_bufs_ = buf_.index;
        buffers_[num_bufs_].bytesused = buf_.bytesused;
        buffers_[num_bufs_].length = buf_.length;
        buffers_[num_bufs_].offset = buf_.m.offset;
        notify();
        
        if(v4l2_ioctl(fd_, VIDIOC_QBUF, &buf_) == -1)
        {
            LOG(LOG_ERROR, "VIDIOC_QBUF error");
            break;
        }
        num_bufs_ = buf_.index;
    }
}

void capture::stop()
{
    is_alive_ = false;
    sleep(1);
    close();
    is_open_ = false;
}

void capture::close()
{
    if (cap_opened_)
    {
        if(is_open_)
        {
            enum v4l2_buf_type type;
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (v4l2_ioctl(fd_, VIDIOC_STREAMOFF, &type))
            {
                LOG(LOG_ERROR, "VIDIOC_STREAMOFF error");
                return;
            }

            for (num_bufs_ = 0; num_bufs_ < cfg_.buff_num; num_bufs_++)
            {
                munmap((void *)(buffers_[num_bufs_].start), buffers_[num_bufs_].length);
                buffers_[num_bufs_].start = nullptr;
            }
            free(buffers_);
            buffers_ = nullptr;
        }
        v4l2_close(fd_);      
    }
}

bool capture::open()
{

    fd_ = v4l2_open(cfg_.dev_name.c_str(), O_RDWR,0);
    if(fd_<0)
    {
        LOG(LOG_ERROR, "open camera: "+cfg_.dev_name+" failed");
        return false;
    }

    /*
    v4l2_capability vc;
    memset(&vc, 0, sizeof(v4l2_capability));
    if(v4l2_ioctl(fd_, VIDIOC_QUERYCAP, &vc) !=-1)
    {
        LOG(LOG_INFO, "driver:\t"<<vc.driver);
        LOG(LOG_INFO, "card:\t"<<vc.card);
        LOG(LOG_INFO, "bus_info:\t"<<vc.bus_info);
        LOG(LOG_INFO, "version:\t"<<vc.version);
    }
    
    v4l2_fmtdesc fmtdesc;
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    LOG(LOG_INFO, "Support format:");
    while (v4l2_ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc) != -1)
    {
        LOG(LOG_INFO, '\t'<<fmtdesc.index<< ". " << fmtdesc.description);
        fmtdesc.index++;
    }
    */
    cap_opened_ = true;
    return true;
}

bool capture::init()
{
    if (format_map_.find(cfg_.format) == format_map_.end())
    {
        LOG(LOG_ERROR, "no supported format");
        return false;
    }
    v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = format_map_[cfg_.format];
    fmt.fmt.pix.width = cfg_.width;
    fmt.fmt.pix.height = cfg_.height;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    
    if (v4l2_ioctl(fd_, VIDIOC_S_FMT, &fmt) == -1)
    {
        LOG(LOG_ERROR, "set format failed");
        return false;
    }
    if (v4l2_ioctl(fd_, VIDIOC_G_FMT, &fmt) == -1)
    {
        LOG(LOG_ERROR, "get format failed");
        return false;
    }
    
    LOG(LOG_INFO, "Capture Image Info:");
    LOG(LOG_INFO, "\tformat:\t"<<(char)(fmt.fmt.pix.pixelformat & 0xFF)<<(char)((fmt.fmt.pix.pixelformat >> 8) & 0xFF)
                <<(char)((fmt.fmt.pix.pixelformat >> 16) & 0xFF)<<(char)((fmt.fmt.pix.pixelformat >> 24) & 0xFF));
    LOG(LOG_INFO, "\twidth:\t"<< fmt.fmt.pix.width);
    LOG(LOG_INFO, "\theight:\t"<< fmt.fmt.pix.height);

    buffer_info_.width = fmt.fmt.pix.width;
    buffer_info_.height = fmt.fmt.pix.height;
    buffer_info_.size = fmt.fmt.pix.sizeimage;
    buffer_info_.format = fmt.fmt.pix.pixelformat;

    v4l2_requestbuffers req;
    req.count = cfg_.buff_num;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (v4l2_ioctl(fd_, VIDIOC_REQBUFS, &req) == -1)
    {
        LOG(LOG_ERROR, "request buffer error ");
        return false;
    }

    buffers_ = (VideoBuffer *)calloc(req.count, sizeof(VideoBuffer));
    for (num_bufs_ = 0; num_bufs_ < req.count; num_bufs_++)
    {
        buf_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf_.memory = V4L2_MEMORY_MMAP;
        buf_.index = num_bufs_;

        if (v4l2_ioctl(fd_, VIDIOC_QUERYBUF, &buf_) == -1)
        {
            LOG(LOG_ERROR, "query buffer error");
            return false;
        }

        buffers_[num_bufs_].length = buf_.length;
        buffers_[num_bufs_].offset = (size_t) buf_.m.offset;
        buffers_[num_bufs_].start = (unsigned char *)mmap(NULL, buf_.length, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd_, buf_.m.offset);
        if (buffers_[num_bufs_].start == MAP_FAILED)
        {
            LOG(LOG_ERROR, "buffer map error");
            return false;
        }
        if (v4l2_ioctl(fd_, VIDIOC_QBUF, &buf_) == -1)
        {
            LOG(LOG_ERROR, "VIDIOC_QBUF error");
            return false;
        }
    }

    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (v4l2_ioctl(fd_, VIDIOC_STREAMON, &type) == -1)
    {
        LOG(LOG_ERROR, "VIDIOC_STREAMON error");
        return false;
    }
    is_open_ = true;
    return true;
}

capture::~capture()
{
    if(td_.joinable()) td_.join();
}
