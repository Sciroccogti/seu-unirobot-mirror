#pragma once

#include <thread>
#include <eigen3/Eigen/Dense>
#include "singleton.hpp"

namespace motion
{
    class ScanEngine: public singleton<ScanEngine>
    {
    public:
        ScanEngine();
        ~ScanEngine();
        void start();
        void stop();

    private:
        void run();
        std::thread td_;
        bool is_alive_;
        float div_;
        Eigen::Vector2f yaw_range_, pitch_range_;
    };

    #define SE ScanEngine::instance()
}