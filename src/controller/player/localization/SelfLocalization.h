#ifndef __SELF_LOCALIZATION_H
#define __SELF_LOCALIZATION_H

#include <iostream>
#include <limits.h>
#include "model.hpp"
#include "kalman.h"
#include "math/math.hpp"
#include "singleton.hpp"

class SelfLocalization: public singleton<SelfLocalization>
{
public:
    struct SLF_conf
    {
        int _sample_num;
        int _div_dir;
        int _observation_update_counter;
    };

    KA _kalman;
    
private:
    SLF_conf _conf;
    bool _haslocated;
public:
    SelfLocalization();
    ~SelfLocalization();

    bool update(const player_info &player_info_,const std::list< GoalPost > & posts_);
    //bool update1(const SensorModel &sensormodel,const Vector3f &moveVec,SoccerMap::PlayerData *player);

    SLF_conf getConfig() const
    {
        return _conf;
    }
};

#define SL SelfLocalization::instance()


#endif