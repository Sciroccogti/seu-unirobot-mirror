#ifndef SEU_UNIROBOT_ACTUATOR_CAPTURE_HPP
#define SEU_UNIROBOT_ACTUATOR_CAPTURE_HPP

#include <map>
#include <thread>
#include <linux/videodev2.h>
#include "image/image_define.hpp"
#include "sensor/sensor.hpp"


class capture: public sensor
{
public: 
    capture(const sub_ptr &s);
    ~capture();
 
    bool start();
    void run();
    void stop();
    bool open();
    void close();
    
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
    
private:
    struct capture_cfg
    {
        std::string camera_name;
        std::string dev_name;
        unsigned int buff_num;
        unsigned int width;
        unsigned int height;
        std::string format;
    };
    
    std::thread td_;
    
    std::map<std::string, unsigned int> format_map_;
    image::VideoBufferInfo buffer_info_;
    image::VideoBuffer *buffers_;
    capture_cfg cfg_;
    v4l2_buffer buf_;
    int num_bufs_;
    int fd_;
    bool cap_opened_;
};

#endif
