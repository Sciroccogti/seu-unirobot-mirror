#ifndef SEU_UNIROBOT_ACTUATOR_CAMERA_HPP
#define SEU_UNIROBOT_ACTUATOR_CAMERA_HPP

#include <map>
#include <thread>
#include <linux/videodev2.h>
#include "image/image_define.hpp"
#include "sensor.hpp"
#include "model.hpp"

class camera: public sensor
{
public:
    struct v4l2_ctrl_item
    {
        v4l2_queryctrl qctrl;
        v4l2_querymenu qmenu[10];
        v4l2_control control;
    };
    camera();
    ~camera();

    bool start();
    void run();
    void stop();
    bool open();
    void close();

    bool set_ctrl_item(const camera_ctrl_info &info);

    image::VideoBufferInfo buff_info() const
    {
        return buffer_info_;
    }

    image::VideoBuffer *buffer() const
    {
        return &(buffers_[num_bufs_]);
    }

private:
    bool init();
    void get_ctrl_items();
    void print_camera_info();
    std::string get_name_by_id(const unsigned int &id);
private:
    struct camera_cfg
    {
        std::string camera_name;
        std::string dev_name;
        unsigned int buff_num;
        unsigned int width;
        unsigned int height;
        std::string format;
        std::string ctrl_file;
    };

    std::thread td_;
    std::vector<camera_ctrl_info> ctrl_infos_;
    std::map<std::string, unsigned int> format_map_;
    image::VideoBufferInfo buffer_info_;
    image::VideoBuffer *buffers_;
    camera_cfg cfg_;
    v4l2_buffer buf_;
    unsigned int num_bufs_;
    int fd_;
    bool cap_opened_;
};

#endif
