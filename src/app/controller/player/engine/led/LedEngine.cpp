#include "LedEngine.hpp"
#include <unistd.h>
#include "common.hpp"
#include "configuration.hpp"

using namespace std;

LedEngine::LedEngine()
{
    led1_ = make_shared<gpio>(gpio::gpio_map[CONF->get_config_value<string>("hardware.gpio.led.1")]);
    led2_ = make_shared<gpio>(gpio::gpio_map[CONF->get_config_value<string>("hardware.gpio.led.2")]);
    can_use_ = (led1_->opened() && led2_->opened());
    if(can_use_)
    {
        led1_->set_direction(gpio::PIN_OUTPUT);
        led2_->set_direction(gpio::PIN_OUTPUT);
    }
    led1_status_ = false;
    led2_status_ = false;
}

LedEngine::~LedEngine()
{
    if(td_.joinable())
    {
        td_.join();
    }
    LOG << std::setw(12) << "engine:" << std::setw(18) << "[LedEngine]" << " ended!" << ENDL;
}

void LedEngine::start()
{
    is_alive_ = true;
    LOG << std::setw(12) << "engine:" << std::setw(18) << "[LedEngine]" << " started!" << ENDL;
    td_ = std::move(std::thread(&LedEngine::run, this));
}

void LedEngine::stop()
{
    is_alive_ = false;
}

void LedEngine::run()
{
    while(is_alive_)
    {
        if(can_use_)
        {
            led1_->set_value(led1_status_?gpio::PIN_LOW:gpio::PIN_HIGH);
            led2_->set_value(led2_status_?gpio::PIN_LOW:gpio::PIN_HIGH);
        }
        usleep(10000);
    }
}

bool LedEngine::set_led(int idx, bool status)
{
    if(idx == 1)
    {
        led1_status_ = status;
        return true;
    }
    else if(idx == 2)
    {
        led2_status_ = status;
        return true;
    }
    else
    {
        return false;
    }
}