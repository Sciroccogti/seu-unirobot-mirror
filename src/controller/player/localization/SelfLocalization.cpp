#include <ctime>
#include <stdlib.h>
#include <limits.h>
#include <boost/foreach.hpp>
#include "SelfLocalization.h"
#include "configuration.hpp"
#include "core/worldmodel.hpp"

using namespace std;

int flag;
SelfLocalization::SelfLocalization(): _haslocated(false)
{
    srand(time(0));

    _conf._sample_num = 60;
    _conf._div_dir = 36;
    _conf._observation_update_counter = 1;
    _kalman.init();
}

SelfLocalization::~SelfLocalization()
{

}


bool SelfLocalization::update(const player_info &player_info_,const list< GoalPost > & posts_)
{
    clock_t start = clock();  
    _kalman.forecast(player_info_);
    KA::State state;
    
    static int obser_count = 0;
    obser_count ++ ;
    
    if (obser_count % _conf._observation_update_counter == 0 )
    {
      //bool check=_kalman.fieldLineUpdate(sensormodel.fieldlines);
      int num=_kalman.goalPostUpdate(posts_);  

      if(flag!=num)
	    _kalman.setPzero();
      if (num==1)
      {
        state=_kalman.obeupdate1();
 
      }

      else if(num==2)
      {
        state=_kalman.obeupdate2();
      }

      flag=num;
    }
    WM->set_my_pos(Eigen::Vector2d(state.x, state.y));

    clock_t finish = clock();
    
    _haslocated = true;
    return true;
}

