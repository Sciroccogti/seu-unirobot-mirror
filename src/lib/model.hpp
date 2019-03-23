#pragma once

#include <string>
#include <map>

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
    bool available = true;
};

struct camera_info
{
    std::string name;
    int id;
    float value;
    float default_value;
    float min_value;
    float max_value;
};

struct camera_param
{
    float fx, fy;
    float cx, cy;
    float k1, k2;
    float p1, p2;
    float h_v, v_v;
};

struct object_prob
{
    int id;
    float prob;
    int x, y;
    object_prob(int i=0, float p=1, int xx=0, int yy=0): id(i), prob(p), x(xx), y(yy){}
};

enum { player_info_size = sizeof(player_info)};
