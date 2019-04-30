#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "singleton.hpp"

namespace motion
{
    enum Head_State
    {
        HEAD_STATE_LOOKAT,
        HEAD_STATE_SEARCH_BALL,
        HEAD_STATE_SEARCH_POST,
        HEAD_STATE_TRACK_BALL
    };

    class ScanEngine: public singleton<ScanEngine>
    {
    public:
        ScanEngine();
        ~ScanEngine();
        void start();
        void stop();
        void set_params(float yaw, float pitch, Head_State state);

        std::atomic_bool search_ball_circle_;
        float lost_yaw_, lost_pitch_;
        
    private:
        void run();
        std::thread td_;
        bool is_alive_;
        float yaw_, pitch_;

        std::atomic_int head_state_;
        const float search_ball_div_ = 3.0;
        const float search_post_div_ = 0.8;
        Eigen::Vector2f pitch_range_;
        std::vector<Eigen::Vector2f> yaw_ranges_;
        Eigen::Vector3f pitches_;
        mutable std::mutex param_mtx_;
    };

    #define SE ScanEngine::instance()
}