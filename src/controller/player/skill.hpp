#pragma once

#include "player.hpp"

extern task_ptr skill_goto(const Eigen::Vector2d &p, double dir, bool omnidirectional=true);
extern task_ptr skill_goto_behind_the_ball(const Eigen::Vector2f &relativeOffset, const Eigen::Vector2f &aimAt);