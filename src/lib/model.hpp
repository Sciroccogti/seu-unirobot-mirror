#pragma once

#include <string>
#include <map>
#include <eigen3/Eigen/Dense>

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

struct ball_block
{
    Eigen::Vector2d global;
    Eigen::Vector2d self;
    Eigen::Vector2d pixel;
    bool sure=true;
};

struct self_block
{
    Eigen::Vector2d global;
    double dir;
    bool sure=true;
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

struct object_det
{
    int id;
    float prob;
    int x, y, w, h;
    object_det(int i=0, float p=1, int x_=0, int y_=0, int w_=0, int h_=0) 
        : id(i), prob(p), x(x_), y(y_), w(w_), h(h_){}
    bool operator< (const object_det &obj)
    {
        return prob<obj.prob;
    }
};

enum { player_info_size = sizeof(player_info)};
