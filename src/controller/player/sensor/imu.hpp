#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include "sensor.hpp"
#include "timer.hpp"

enum led_state
{
    LED_NORMAL = 1,
    LED_WARNING = 2,
    LED_ERROR = 3
};

struct sw_data
{
    bool sw1 = false;
    bool sw2 = false;
};

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

    inline void set_led_state(const led_state &s)
    {
        led_mtx_.lock();
        l_state_ = s;
        led_mtx_.unlock();
    }
    inline void set_zero()
    {
        led_mtx_.lock();
        reset_ = true;
        led_mtx_.unlock();
    }
    inline imu_data data() const
    {
        return imu_data_;
    }
    inline bool lost() const
    {
        return lost_;
    }
    inline sw_data switch_data() const
    {
        return sw_data_;
    }
private:
    bool open();
    void read_head0();
    void read_head1();
    void read_data();
    void run();

    enum {imu_data_size = sizeof(imu_data)};
    enum {imu_len = 11};
    unsigned char buff_[imu_len];
    imu_data imu_data_;
    sw_data sw_data_;
    std::thread td_;
    std::mutex led_mtx_;
    boost::asio::serial_port serial_;
    unsigned char led_;
    led_state l_state_;
    bool reset_;
    std::atomic_bool lost_;
    std::atomic_bool connected_;
    std::atomic_int count_;
};
