#ifndef SEU_UNIROBOT_ACTUATOR_WORLD_MODEL_HPP
#define SEU_UNIROBOT_ACTUATOR_WORLD_MODEL_HPP

#include "singleton.hpp"

class world_model: public singleton<world_model>
{

};

#define WM world_model::get_singleton()

#endif
