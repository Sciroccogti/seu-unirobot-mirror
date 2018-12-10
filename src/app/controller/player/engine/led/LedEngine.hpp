#pragma once

#include <thread>
#include <memory>
#include <atomic>
#include "singleton.hpp"
#include "gpio/gpio.hpp"


class LedEngine: public singleton<LedEngine>
{
public:
    LedEngine();
    ~LedEngine();
    void start();
    void stop();
    bool set_led(int idx, bool status);
    
private:
    void run();
    std::thread td_;
    bool is_alive_;
    bool can_use_;
    std::atomic_bool led1_status_;
    std::atomic_bool led2_status_;

    std::shared_ptr<gpio> led1_;
    std::shared_ptr<gpio> led2_;
};

#define LE LedEngine::instance()