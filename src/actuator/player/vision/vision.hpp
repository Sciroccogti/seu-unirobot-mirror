#ifndef SEU_UNIROBOT_ACTUATOR_VISION_HPP
#define SEU_UNIROBOT_ACTUATOR_VISION_HPP

#include <mutex>
#include <opencv2/opencv.hpp>
#include "image/image_define.hpp"
#include "timer.hpp"
#include "pattern.hpp"
#include "capture.hpp"
#include "sensor/tcp_server.hpp"

class vision: public timer, public subscriber
{
public:
    vision(const sensor_ptr &s=nullptr);
    ~vision();
    void updata(const pub_ptr &pub);
    
    bool start();
    void stop();
    mutable std::mutex frame_mutex_;
private:
    void run();
    std::shared_ptr<capture> cap_;
    cv::Mat frame_;
    std::shared_ptr<tcp_server> server_;
    int width_;
    int height_;
    int p_count_;
};

#endif