#ifndef SEU_UNIROBOT_ACTUATOR_IMU_HPP
#define SEU_UNIROBOT_ACTUATOR_IMU_HPP

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include "sensor.hpp"

class imu: public sensor
{
public:
    struct imu_data
    {
        float pitch, roll, yaw;
        float ax, ay, az;
        float wx, wy, wz;
    };
    
    imu(const sub_ptr &s);
    ~imu();
    
    bool start();
    void stop();
    
    void data_handler(const char* data, const int& size, const int& type=0);
    
    imu_data data() const
    {
        return *data_;
    }
    
private:
    bool open();
    
    enum {imu_data_size = sizeof(imu_data)};
    imu_data *data_;
    std::string str_;
    std::thread td_;
    boost::asio::serial_port serial_;
};

#endif