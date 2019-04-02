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
    div_ = CONF->get_config_value<float>("scan.div");
    yaw_ = 0.0;
    pitch_ = 0.0;
    scan_ = false;
    pitches_[0] = pitch_range_[0];
    pitches_[1] = pitch_range_[0]+(pitch_range_[1]-pitch_range_[0])/2.0f;
    pitches_[2] = pitch_range_[1];
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

void ScanEngine::set_params(float yaw, float pitch, bool scan)
{
    param_mtx_.lock();
    yaw_ = yaw;
    pitch_ = pitch;
    scan_ = scan;
    param_mtx_.unlock();
}

void ScanEngine::run()
{
    unsigned int line = 0;
    int id_yaw = ROBOT->get_joint("jhead1")->jid_;
    int id_pitch = ROBOT->get_joint("jhead2")->jid_;
    std::map<int, float> jdmap;
    jdmap[id_yaw] = 0.0;
    jdmap[id_pitch] = 0.0;
    float yaw, pitch;
    bool scan;
    bool search_ball=true;
    bool search_post=false;
    while(is_alive_)
    {
        ball_block ball = WM->ball();
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
        if(search_post)
        {
            float search_div = 1.2;
            for(int i=1;i>=0;i--)
            {
                for(float ya=post_search_table[i*2][1]; fabs(ya)<=fabs(post_search_table[i*2+1][1])+0.1;ya+=pow(-1, i+1)*search_div)
                {
                    jdmap[id_pitch] = post_search_table[i*2][0];
                    jdmap[id_yaw] = ya;
                    while (!MADT->head_empty())
                    {
                        usleep(1000);
                    }
                    if (!MADT->add_head_degs(jdmap))
                    {
                        break;
                    }
                    if(WM->find_two_posts) break;
                }
                if(WM->find_two_posts) break;
            }
        }
        else
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
        /*
        {
            int idx = line%4-2;
            if(idx<0) idx = -idx;
            jdmap[id_pitch] = pitches_[idx];
            float s = pow(-1, idx);
            Vector2f yaw_range_=yaw_ranges_[idx];
            for(float yawt = yaw_range_[0]; yawt <= yaw_range_[1]&&is_alive_; yawt += div_)
            {
                jdmap[id_yaw] = s*yawt;
                while (!MADT->head_empty())
                {
                    usleep(1000);
                }
                if (!MADT->add_head_degs(jdmap))
                {
                    break;
                }
            }
            line++;
        }
        
        else*/
        /*
        param_mtx_.lock();
        yaw = yaw_;
        pitch = pitch_;
        param_mtx_.unlock();
        */
       /*
        std::vector<double> head_degs = ROBOT->get_head_degs();
        yaw = head_degs[0];

        if (fabs(ball.alpha) > 0.1f)
        {
            yaw += -(sign(ball.alpha) * 5.0f);
        }
            
        if (ball.self.x() < 0.8f)
        {
            pitch = 60.0;
        }
        else
        {
            pitch = head_degs[1];
        }
        {
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
        }*/
        usleep(500);
    }
}
}