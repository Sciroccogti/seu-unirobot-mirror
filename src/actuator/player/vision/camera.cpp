#include <libv4l2.h>
#include <sys/mman.h>
#include <errno.h>
#include <functional>
#include "camera.hpp"
#include "configuration.hpp"
#include "class_exception.hpp"
#include "parser/camera_parser.hpp"

using namespace std;
using namespace image;

camera::camera(): sensor("camera")
{
    format_map_["V4L2_PIX_FMT_YUYV"] = V4L2_PIX_FMT_YUYV;
    format_map_["V4L2_PIX_FMT_MJPEG"] = V4L2_PIX_FMT_MJPEG;
    format_map_["V4L2_PIX_FMT_JPEG"] = V4L2_PIX_FMT_JPEG;
    cfg_.dev_name = CONF->get_config_value<string>("video.dev_name");
    cfg_.buff_num = CONF->get_config_value<int>("video.buff_num");
    cfg_.format = CONF->get_config_value<string>("video.format");
    cfg_.height = CONF->get_config_value<int>("video.height");
    cfg_.width = CONF->get_config_value<int>("video.width");
    cfg_.ctrl_file = CONF->get_config_value<string>(CONF->player()+".camera_file");
}

bool camera::start()
{
    if(!open()) return false;
    if(!init()) return false;
    td_ = thread(bind(&camera::run, this));
    return true;
}

void camera::run()
{
    is_alive_ = true;
    buf_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf_.memory = V4L2_MEMORY_MMAP;

    while(is_alive_)
    {
        if (v4l2_ioctl(fd_, VIDIOC_DQBUF, &buf_) == -1)
        {
            std::cout<<"VIDIOC_DQBUF failed.\n";
            break;
        }
        num_bufs_ = buf_.index;
        buffers_[num_bufs_].bytesused = buf_.bytesused;
        buffers_[num_bufs_].length = buf_.length;
        buffers_[num_bufs_].offset = buf_.m.offset;
        notify(SENSOR_CAMERA);
        
        if(v4l2_ioctl(fd_, VIDIOC_QBUF, &buf_) == -1)
        {
            std::cout<<"VIDIOC_QBUF error\n";
            break;
        }
        num_bufs_ = buf_.index;
    }
}

void camera::stop()
{
    is_alive_ = false;
    sleep(1);
    close();
    is_open_ = false;
}

void camera::get_ctrl_items()
{
    ctrl_items_.clear();
    camera::v4l2_ctrl_item item;
    item.qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    while(v4l2_ioctl (fd_, VIDIOC_QUERYCTRL, &item.qctrl) == 0)
    {
        if (item.qctrl.type == V4L2_CTRL_TYPE_MENU)
        {
            auto &ctrl = item.qctrl;
            const int maxmenu_size = sizeof(item.qmenu) / sizeof(v4l2_querymenu);
            for (int i = ctrl.minimum; i <= std::min(ctrl.maximum, maxmenu_size); i++)
            {
                auto &qm = item.qmenu[i];
                qm.id = ctrl.id;
                qm.index = i;
                if (v4l2_ioctl(fd_, VIDIOC_QUERYMENU, &qm) != 0)
                    std::strcpy((char *)(qm.name), "Unknown");
            }
        }
        item.control.id = item.qctrl.id;
        get_ctrl_item(item);
        ctrl_items_.push_back(item);
        item.qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
}

bool camera::get_ctrl_item(camera::v4l2_ctrl_item &item)
{
    if (fd_ < 0) return false;

    v4l2_queryctrl ctrl;
    ctrl.id = item.qctrl.id;
    if (v4l2_ioctl(fd_, VIDIOC_QUERYCTRL, &ctrl) == -1)
    {
        std::cout << "Invalid control item " << ctrl.name << std::endl;
        return false;
    }
    if (ctrl.type == V4L2_CTRL_TYPE_BUTTON) return true;
    if (v4l2_ioctl(fd_, VIDIOC_G_CTRL, &(item.control)) == -1)
    {
        std::cout << "Unable to get " << ctrl.name << " " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool camera::set_ctrl_item(const camera::v4l2_ctrl_item &item)
{
    if (fd_ < 0) return false;

    v4l2_queryctrl ctrl;
    ctrl.id = item.qctrl.id;

    if (v4l2_ioctl(fd_, VIDIOC_QUERYCTRL, &ctrl) == -1)
    {
        std::cout << "Invalid control item " << ctrl.name << std::endl;
        return false;
    }

    if (ctrl.flags & (V4L2_CTRL_FLAG_READ_ONLY |
                        V4L2_CTRL_FLAG_DISABLED |
                        V4L2_CTRL_FLAG_GRABBED))
    {
        return true;
    }

    if (ctrl.type != V4L2_CTRL_TYPE_INTEGER &&
        ctrl.type != V4L2_CTRL_TYPE_BOOLEAN &&
        ctrl.type != V4L2_CTRL_TYPE_MENU)
    {
        return true;
    }

    auto c = item.control;
    if (v4l2_ioctl(fd_, VIDIOC_S_CTRL, &c) == -1)
    {
        std::cout << "Unable to set control " << ctrl.name << " " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void camera::close()
{
    if (cap_opened_)
    {
        if(is_open_)
        {
            enum v4l2_buf_type type;
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (v4l2_ioctl(fd_, VIDIOC_STREAMOFF, &type))
            {
                std::cout<<"VIDIOC_STREAMOFF error\n";
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

bool camera::open()
{

    fd_ = v4l2_open(cfg_.dev_name.c_str(), O_RDWR,0);
    if(fd_<0)
    {
        std::cout<<"open camera: "+cfg_.dev_name+" failed\n";
        return false;
    }

    /*
    v4l2_capability vc;
    memset(&vc, 0, sizeof(v4l2_capability));
    if(v4l2_ioctl(fd_, VIDIOC_QUERYCAP, &vc) !=-1)
    {
        std::cout<<"driver:\t"<<vc.driver<<"\n";
        std::cout<<"card:\t"<<vc.card<<"\n";
        std::cout<<"bus_info:\t"<<vc.bus_info<<"\n";
        std::cout<<"version:\t"<<vc.version<<"\n";
    }
    
    v4l2_fmtdesc fmtdesc;
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_camera;
    std::cout<<"Support format:\n";
    while (v4l2_ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc) != -1)
    {
        std::cout<<'\t'<<fmtdesc.index<< ". " << fmtdesc.description<<"\n";
        fmtdesc.index++;
    }
    */
    cap_opened_ = true;
    return true;
}

bool camera::init()
{
    if (format_map_.find(cfg_.format) == format_map_.end())
    {
        std::cout<<"no supported format\n";
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
        std::cout<<"set format failed\n";
        return false;
    }
    if (v4l2_ioctl(fd_, VIDIOC_G_FMT, &fmt) == -1)
    {
        std::cout<<"get format failed\n";
        return false;
    }
    std::cout<<"\033[32m--------------------------------------------------------\n";
    std::cout<<"Image Info: [";
    std::cout<<"format: "<<(char)(fmt.fmt.pix.pixelformat & 0xFF)<<(char)((fmt.fmt.pix.pixelformat >> 8)
        & 0xFF)<<(char)((fmt.fmt.pix.pixelformat >> 16) & 0xFF)<<(char)((fmt.fmt.pix.pixelformat >> 24) & 0xFF);
    std::cout<<" width: "<< fmt.fmt.pix.width;
    std::cout<<" height: "<< fmt.fmt.pix.height<<"]\n";
    std::cout<<"--------------------------------------------------------\n\033[0m";
    
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
        std::cout<<"request buffer error \n";
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
            std::cout<<"query buffer error\n";
            return false;
        }

        buffers_[num_bufs_].length = buf_.length;
        buffers_[num_bufs_].offset = (size_t) buf_.m.offset;
        buffers_[num_bufs_].start = (unsigned char *)mmap(NULL, buf_.length, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd_, buf_.m.offset);
        if (buffers_[num_bufs_].start == MAP_FAILED)
        {
            std::cout<<"buffer map error\n";
            return false;
        }
        if (v4l2_ioctl(fd_, VIDIOC_QBUF, &buf_) == -1)
        {
            std::cout<<"VIDIOC_QBUF error\n";
            return false;
        }
    }

    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (v4l2_ioctl(fd_, VIDIOC_STREAMON, &type) == -1)
    {
        std::cout<<"VIDIOC_STREAMON error\n";
        return false;
    }
    get_ctrl_items();
    vector<ctrl_item_info> ctrl_items;
    parser::camera_parser::parse(cfg_.ctrl_file, ctrl_items);
    for(auto &it: ctrl_items)
    {
        for(auto &item: ctrl_items_)
            if(item.control.id == it.id)
                item.control.value = it.value;
    }
    for(auto &item: ctrl_items_)
        set_ctrl_item(item);
    is_open_ = true;
    return true;
}

camera::~camera()
{
    if(td_.joinable()) td_.join();
}
