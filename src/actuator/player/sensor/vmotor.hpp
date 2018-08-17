#ifndef SEU_UNIROBOT_ACTUATOR_VMOTOR_HPP
#define SEU_UNIROBOT_ACTUATOR_VMOTOR_HPP

#include <memory>
#include <list>
#include <mutex>
#include "debuger.hpp"
#include "motor.hpp"
#include "robot/humanoid.hpp"


class vmotor: public motor
{
public:
    vmotor(const sub_ptr &s, sensor_ptr dbg=nullptr);
    ~vmotor();

    void act();

private:
    std::shared_ptr<debuger> dbg_;
};

#endif