#pragma once

#include <string>

struct filed_info
{
    int field_length;
    int field_width;
    int goal_depth;
    int goal_width;
    int goal_height;
    int goal_area_length;
    int goal_area_width;
    int penalty_mark_distance;
    int center_circle_diameter;
    int border_strip_width_min;
};

struct player_info
{
    int id;
    float x, y, dir;
    float ball_x, ball_y;
};

struct camera_para
{
    std::string name;
    int id;
    float value;
    float default_value;
    float min_value;
    float max_value;
};

enum { player_info_size = sizeof(player_info)};
