#include "ScanEngine.hpp"
#include "configuration.hpp"
#include "robot/humanoid.hpp"
#include "core/adapter.hpp"
#include "math/math.hpp"
#include <cmath>
#include "logger.hpp"
#include "core/worldmodel.hpp"

using namespace std;
using namespace robot;
using namespace robot_math;
using namespace Eigen;

namespace motion
{
const float skill_head_pitch_min_angle = 0.0f;
const float skill_head_pitch_mid_angle = 30.0f;
const float skill_head_pitch_max_angle = 60.0f;

const int scan_ball =10;
const int scan_post = 8;
const float ball_search_table[][2] =
{
    {skill_head_pitch_min_angle, 85.0}, 
    {skill_head_pitch_min_angle, 30.0},
    {skill_head_pitch_min_angle, -30.0}, 
    {skill_head_pitch_min_angle, -85.0},

    {skill_head_pitch_mid_angle, -75.0},
    {skill_head_pitch_mid_angle, -25.0},
    {skill_head_pitch_mid_angle, 25.0},
    {skill_head_pitch_mid_angle, 75.0},
    
    {skill_head_pitch_max_angle, 30.0}, 
    {skill_head_pitch_max_angle, -30.0}
};

const float post_search_table[][2] = 
{
    {skill_head_pitch_min_angle, 90.0},
    {skill_head_pitch_min_angle, -90.0},
    {skill_head_pitch_mid_angle, -90.0},
    {skill_head_pitch_mid_angle, 90.0}
};

ScanEngine::ScanEngine()
{
    std::vector<float> range = CONF->get_config_vector<float>("scan.pitch");
    pitch_range_[0] = range[0];
    pitch_range_[1] = range[1];
    range = CONF->get_config_vector<float>("scan.yaw");
    yaw_ranges_.push_back(Vector2f(range[0], range[1]));
    yaw_ranges_.push_back(Vector2f(range[0]+30.0, range[1]-30.0));
    yaw_ranges_.push_back(Vector2f(range[0]+60.0, range[1]-60.0));
    pitches_[0] = pitch_range_[0];
    pitches_[1] = pitch_range_[0]+(pitch_range_[1]-pitch_range_[0])/2.0f;
    pitches_[2] = pitch_range_[1];
    head_state_ = HEAD_STATE_LOOKAT;
    vector<float> head_init = CONF->get_config_vector<float>("scan.init");

    yaw_ = head_init[0];
    pitch_ = head_init[1];
    search_ball_circle_ = false;
}

ScanEngine::~ScanEngine()
{
    if(td_.joinable())
    {
        td_.join();
    }
    LOG(LOG_INFO) << std::setw(12) << "engine:" << std::setw(18) << "[ScanEngine]" << " ended!" << endll;
}

void ScanEngine::start()
{
    LOG(LOG_INFO) << std::setw(12) << "engine:" << std::setw(18) << "[ScanEngine]" << " started!" << endll;
    is_alive_ = true;
    td_ = std::move(std::thread(&ScanEngine::run, this));
}

void ScanEngine::stop()
{
    is_alive_ = false;
}

void ScanEngine::set_params(float yaw, float pitch, Head_State state)
{
    param_mtx_.lock();
    yaw_ = yaw;
    pitch_ = pitch;
    head_state_ = state;
    param_mtx_.unlock();
}

void ScanEngine::run()
{
    int id_yaw = ROBOT->get_joint("jhead1")->jid_;
    int id_pitch = ROBOT->get_joint("jhead2")->jid_;
    std::map<int, float> jdmap;
    jdmap[id_yaw] = 0.0;
    jdmap[id_pitch] = 0.0;
    float yaw, pitch;
    while(is_alive_)
    {
        ball_block ball = WM->ball();
        if(head_state_ == HEAD_STATE_SEARCH_BALL)
        {
            WM->self_localization_ = false;
            for(int i=scan_ball-1;i>=0&&!ball.can_see;i--)
            {
                jdmap[id_pitch] = ball_search_table[i][0];
                jdmap[id_yaw] = ball_search_table[i][1];
                while (!MADT->head_empty())
                {
                    usleep(1000);
                }
                if (!MADT->add_head_degs(jdmap))
                {
                    break;
                }
                usleep(1000000);
                ball = WM->ball();
            }
            search_ball_circle_ = true;
        }
        else if(head_state_ == HEAD_STATE_SEARCH_POST)
        {
            WM->self_localization_ = true;
            for(int i=1;i>=0;i--)
            {
                jdmap[id_pitch] = post_search_table[i*2][0];
                for(float ya=post_search_table[i*2][1]; !WM->can_see_post_ && fabs(ya)<=fabs(post_search_table[i*2+1][1])+0.1; ya+=pow(-1, i+1)*search_post_div_)
                {
                    jdmap[id_yaw] = ya;
                    while (!MADT->head_empty())
                    {
                        usleep(1000);
                    }
                    if (!MADT->add_head_degs(jdmap))
                    {
                        break;
                    }
                }
            }
            WM->self_localization_ = false;
            WM->in_localization_ = false;
            LOG(LOG_INFO)<<"localization end"<<endll;
            param_mtx_.lock();
            yaw_ = lost_yaw_;
            pitch_ = lost_pitch_;
            head_state_ = HEAD_STATE_LOOKAT;
            param_mtx_.unlock();
        }
        else
        {
            WM->self_localization_ = false;
            if(head_state_ == HEAD_STATE_TRACK_BALL)
            {
                std::vector<double> head_degs = ROBOT->get_head_degs();
                yaw = head_degs[0];
                pitch = head_degs[1];
                if (fabs(ball.alpha) > 0.2f)
                {
                    yaw += -(sign(ball.alpha)*1.0f);
                }  
                if (fabs(ball.beta) > 0.2f)
                {
                    pitch += sign(ball.beta)*1.0f;
                }

                bound(yaw_ranges_[1][0], yaw_ranges_[1][1], yaw);
                bound(pitch_range_[0], pitch_range_[1], pitch);
                jdmap[id_yaw] = yaw;
                jdmap[id_pitch] = pitch;

                while (!MADT->head_empty())
                {
                    usleep(1000);
                }
                if (!MADT->add_head_degs(jdmap))
                {
                    break;
                }
            }
            else if(head_state_ == HEAD_STATE_LOOKAT)
            {
                param_mtx_.lock();
                yaw = yaw_;
                pitch = pitch_;
                param_mtx_.unlock();
                bound(yaw_ranges_[1][0], yaw_ranges_[1][1], yaw);
                bound(pitch_range_[0], pitch_range_[1], pitch);
                jdmap[id_yaw] = yaw;
                jdmap[id_pitch] = pitch;
                while (!MADT->head_empty())
                {
                    usleep(1000);
                }
                if (!MADT->add_head_degs(jdmap))
                {
                    break;
                }
            }
        }
        usleep(500);
    }
}
}