#ifndef SEU_UNIROBOT_ACTUATOR_VISION_HPP
#define SEU_UNIROBOT_ACTUATOR_VISION_HPP

#include <mutex>
#include <opencv2/opencv.hpp>
#include "image/image_define.hpp"
#include "timer.hpp"
#include "pattern.hpp"
#include "camera.hpp"
#include "sensor/server.hpp"

class vision: public timer, public subscriber
{
public:
    vision(const sensor_ptr &s=nullptr);
    ~vision();
    void updata(const pro_ptr &pub, const int &type);
    
    bool start();
    void stop();
    mutable std::mutex frame_mutex_;
private:
    void run();
    void send_image(const cv::Mat &yuvsrc);
    cv::Mat frame_;
    std::shared_ptr<tcp_server> server_;
    int p_count_;
};

#endif