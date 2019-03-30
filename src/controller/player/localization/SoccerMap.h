#ifndef __SOCCER_MAP_H
#define __SOCCER_MAP_H

#include <map>
#include "singleton.hpp"
#include "math/Math.hpp"
#include "configuration.hpp"
#include "model.hpp"

#define SIMULATION_HEIGHT    0.7f

class SoccerMap : public singleton<SoccerMap>
{
public:
    enum GoalPostType
    {
        FLAG_DOOR_OPP_P,
        FLAG_DOOR_OPP_N,
        FLAG_DOOR_OUR_P,
        FLAG_DOOR_OUR_N,
    };

    struct PlayerData
    {
        float x, y;
        seumath::AngDeg bodydir;
        seumath::AngDeg camdir;
        seumath::AngDeg camWidth;
        float camDis;
    };

private:
    bool getConfigInfo();
private:
    //base dimensions.
    int _field_length;
    int _field_width;
    int _goal_depth;
    int _goal_width;
    int _goal_height;
    int _goal_area_length;
    int _goal_area_width;
    int _penalty_mark_distance;
    int _center_circle_diameter;
    int _border_strip_width_min;
    filed_info filed_info_;

    //some intermediate variable.
    seumath::Vector2i _center;
    std::map<GoalPostType, seumath::Vector2f> _goal_post_position;

    int _xMax, _xMin, _yMax, _yMin;
    seumath::Vector2f _opp_goal, _our_goal;

public:
    SoccerMap();

    int width() const
    {
        return _field_length;
    }

    int height() const
    {
        return _field_width;
    }

    seumath::Vector2i center() const
    {
        return _center;
    }

    int goal_area_length() const
    {
        return _goal_area_length;
    }

    int border_strip_width_min() const
    {
        return _border_strip_width_min;
    }

    int goal_area_width() const
    {
        return _goal_area_width;
    }

    int center_circle_diameter() const
    {
        return _center_circle_diameter;
    }

    int penalty_mark_distance() const
    {
        return _penalty_mark_distance;
    }


    std::map<GoalPostType, seumath::Vector2f> flagPos() const
    {
        return _goal_post_position;
    }

    int maxX() const
    {
        return _xMax;
    }

    int minX() const
    {
        return _xMin;
    }

    int maxY() const
    {
        return _yMax;
    }

    int minY() const
    {
        return _yMin;
    }

    seumath::Vector2f ourGoal() const
    {
        return _our_goal;
    }

    seumath::Vector2f ourGoalL() const
    {
        return (_goal_post_position.at(FLAG_DOOR_OUR_P));
    }

    seumath::Vector2f outGoalR() const
    {
        return (_goal_post_position.at(FLAG_DOOR_OUR_N));
    }

    seumath::Vector2f oppGoal() const
    {
        return _opp_goal;
    }

    seumath::Vector2f oppGoalL() const
    {
        return (_goal_post_position.at(FLAG_DOOR_OPP_P));
    }

    seumath::Vector2f oppGoalR() const
    {
        return (_goal_post_position.at(FLAG_DOOR_OPP_N));
    }

    float goalWidth() const
    {
        return _goal_width;
    }


};

#define SOCCERMAP SoccerMap::instance()
#define DEBUG_SOCCER_MAP SoccerMap::instance()->_debugSoccerMap

#endif