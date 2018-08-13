#ifndef SEU_UNIROBOT_IMU_HPP
#define SEU_UNIROBOT_IMU_HPP

#include <string>
#include <thread>
#include "sensor.hpp"
#include "comm/serial_port.hpp"

class imu: public sensor
{
public:
    struct imu_data
    {
        float pitch, roll, yaw;
        float ax, ay, az;
    };
    
    imu(const sub_ptr &s);
    ~imu();
    
    bool start();
    void run();
    bool open();
    void close();
    
    void data_handler(const char* data, const int& size, const int& type=0);
    
    imu_data data() const
    {
        return *data_;
    }
    
private:
    enum {imu_data_size = sizeof(imu_data)};
    imu_data *data_;
    std::string str_;
    std::thread td_;
    comm::serial_port serial_;
};

#endif