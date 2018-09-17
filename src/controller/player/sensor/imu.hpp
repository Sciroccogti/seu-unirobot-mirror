#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include "sensor.hpp"
#include "timer.hpp"

class imu: public sensor, public timer
{
public:
    struct imu_data
    {
        float pitch, roll, yaw;
        float ax, ay, az;
        float wx, wy, wz;
    };

    imu();
    ~imu();

    bool start();
    void stop();
    void set_led(unsigned char led1=0, unsigned char led2=0);

    imu_data data() const
    {
        return data_;
    }

private:
    bool open();
    void read_head0();
    void read_head1();
    void read_data();
    void io_init();
    void run();
    enum led_mode
    {
        LED_NORMAL = 1,
        LED_ERROR = 2
    };

    enum {imu_data_size = sizeof(imu_data)};
    enum {imu_len = 11};
    unsigned char buff_[imu_len];
    imu_data data_;
    std::thread td_;
    std::mutex led_mtx_;
    boost::asio::serial_port serial_;
    unsigned char led1_, led2_;
    led_mode l_mode_;
};
