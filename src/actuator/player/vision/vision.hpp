#ifndef SEU_UNIROBOT_ACTUATOR_VISION_HPP
#define SEU_UNIROBOT_ACTUATOR_VISION_HPP

#include <mutex>
#include <opencv2/opencv.hpp>
#include "image/image_define.hpp"
#include "timer.hpp"
#include "pattern.hpp"
#include "capture.hpp"
#include "sensor/server.hpp"

class vision: public timer, public subscriber
{
public:
    vision(const sensor_ptr &s=nullptr);
    ~vision();
    void updata(const pub_ptr &pub);
    void run();
    
    bool start();
    void stop();
    mutable std::mutex frame_mutex_;
private:
    std::shared_ptr<capture> cap_;
    //image::VideoFrame_Ptr frame_;
    cv::Mat frame_;
    std::shared_ptr<server> server_;
    const int width = 640;
    const int height = 480;
    int p_count_;
};

#endif