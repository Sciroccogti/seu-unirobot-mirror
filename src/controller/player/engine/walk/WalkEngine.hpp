#pragma once

#include <map>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include "observer.hpp"
#include "sensor/imu.hpp"
#include "robot/robot_define.hpp"
#include "singleton.hpp"

namespace motion
{
    class WalkEngine: public subscriber, public singleton<WalkEngine>
    {
    public:
        WalkEngine();
        ~WalkEngine();
        void start();
        void stop()
        {
            is_alive_ = false;
        }
        void set_params(float x, float y, float d, bool enable);
        void updata(const pub_ptr &pub, const int &type);

    private:
        static void boundPhase(double &phase);
        

        double x0_, y0_;
        double xt_, yt_, dt_;
        bool enable_;
        float support_foot;
        double Cz_, wn_;
        double freq_;
        double T_;
        double Tm_;
        double Td_;
        double h_;
        double footYoffset_;
        
        void run();
        std::thread td_;
        bool is_alive_;
        Eigen::Vector2d xrange, yrange, drange;
        imu::imu_data imu_data_;
        mutable std::mutex para_mutex_, imu_mtx_, dxl_mtx_;
    };

#define WE WalkEngine::instance()
}

