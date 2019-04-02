#include "WalkEngine.hpp"
#include "CubicSpline.hpp"
#include "SmoothSpline.hpp"
#include "Polynom.hpp"
#include "robot/humanoid.hpp"
#include "math/math.hpp"
#include "configuration.hpp"
#include <cmath>
#include <fstream>
#include "core/adapter.hpp"
#include "sensor/motor.hpp"
#include "core/worldmodel.hpp"
#include "logger.hpp"

namespace motion
{
    using namespace Eigen;
    using namespace robot;
    using namespace robot_math;
    using namespace std;

    WalkEngine::WalkEngine()
    {
        string part=CONF->player()+".walk";
        std::vector<double> range = CONF->get_config_vector<double>(part+".x");
        xrange[0] = range[0];
        xrange[1] = range[1];
        range = CONF->get_config_vector<double>(part+".y");
        yrange[0] = range[0];
        yrange[1] = range[1];
        range = CONF->get_config_vector<double>(part+".dir");
        drange[0] = range[0];
        drange[1] = range[1];

        x0_ = 0.0;
        y0_ = 0.0;
        support_foot = -1.0;
        Cz_ = ROBOT->leg_length()-0.02;
        wn_ = sqrt(9.8/Cz_);
        freq_ = 1.5;
        T_ = 1.0/freq_/2.0;
        Tm_ = 0.02;
        Td_ = 0.05*T_;
        h_ = 0.04;
        footYoffset_ = 0.04;
    }

    void WalkEngine::updata(const pub_ptr &pub, const int &type)
    {
        if (type == sensor::SENSOR_IMU)
        {
            std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
            imu_mtx_.lock();
            imu_data_ = sptr->data();
            imu_mtx_.unlock();
            return;
        }

        if (type == sensor::SENSOR_MOTOR)
        {
            std::shared_ptr<motor> sptr = std::dynamic_pointer_cast<motor>(pub);
            dxl_mtx_.lock();
            
            dxl_mtx_.unlock();
            return;
        }
    }

    WalkEngine::~WalkEngine()
    {
        if (td_.joinable())
        {
            td_.join();
        }
        LOG(LOG_INFO) << std::setw(12) << "engine:" << std::setw(18) << "[WalkEngine]" << " ended!" << endll;
    }

    void WalkEngine::start()
    {
        dt_ = 1.0 / (1000.0 / CONF->get_config_value<double>("hardware.motor.period"));
        is_alive_ = true;
        td_ = std::move(std::thread(&WalkEngine::run, this));
        LOG(LOG_INFO) << std::setw(12) << "engine:" << std::setw(18) << "[WalkEngine]" << " started!" << endll;
    }

    void WalkEngine::boundPhase(double &phase)
    {
        while (phase >= 1.0)
        {
            phase -= 1.0;
            if (phase < 0.0)
            {
                phase = 0.0;
            }
        }
    }

    void WalkEngine::set_params(float x, float y, float d, bool enable)
    {
        para_mutex_.lock();
        xt_ = x;
        yt_ = y;
        dt_ = d;
        enable_ = enable;
        para_mutex_.unlock();
    }

    void WalkEngine::run()
    {   
        transform_matrix body_mat, leftfoot_mat, rightfoot_mat, swingfoot_mat, supportfoot_mat;
        Quaternion<double> quat;
        AngleAxisd yawRot, pitchRot, rollRot;
        
        std::vector<double> degs;
        std::map<int, float> jdegs;
        double xt, yt, dt;
        bool e;

        while (is_alive_)
        {   
            jdegs.clear();
            para_mutex_.lock();
            xt = xt_;
            yt = yt_;
            dt = dt_;
            e = enable_;
            para_mutex_.unlock();

            if (MADT->get_mode() == adapter::MODE_READY)
            {
                xt_ = 0.0;
                yt_ = 0.0;
                dt_ = 0.0;
            }
            if(e&&(MADT->get_mode() == adapter::MODE_READY || MADT->get_mode() == adapter::MODE_WALK))
            {             
                SmoothSpline stepSpline;
                stepSpline.addPoint(0.0, x0_);
                stepSpline.addPoint(Td_, x0_);
                stepSpline.addPoint(T_-Td_, xt);
                stepSpline.addPoint(T_, xt);

                SmoothSpline laterSpline;
                laterSpline.addPoint(0.0, y0_);
                laterSpline.addPoint(Td_, y0_);
                laterSpline.addPoint(T_-Td_, yt);
                laterSpline.addPoint(T_, yt);

                SmoothSpline riseSpline;
                riseSpline.addPoint(0.0, 0.0);
                riseSpline.addPoint(Td_, 0.0);
                riseSpline.addPoint(T_ / 2.0, h_);
                riseSpline.addPoint(T_-Td_, 0.0);
                riseSpline.addPoint(T_, 0.0);

                SmoothSpline turnSpline;
                turnSpline.addPoint(0.0, 0.0);
                turnSpline.addPoint(Td_, 0.0);
                turnSpline.addPoint(T_-Td_, deg2rad(dt));
                turnSpline.addPoint(T_, deg2rad(dt));  

                double B = (xt-x0_)/2.0, A = support_foot* ROBOT->D()/2.0;
                double Kx = B*Td_*wn_/(Td_*wn_+tanh(wn_*(T_/2-Td_)));
                SmoothSpline bodyXSpline;
                bodyXSpline.addPoint(0.0, x0_/2.0, Kx/Td_);
                bodyXSpline.addPoint(T_, xt/2.0, Kx/Td_);

                double Ky = A * Td_ * wn_ * tanh(wn_ * (T_ / 2 - Td_)) / (1 + Td_ * wn_ * tanh(wn_ * (T_/2 - Td_)));
                double Cy1 = Ky - A, Cy2 = Ky / (Td_ * wn_);
                float t;
                vector<float> Cy;
                Cy.clear();
                for (t = 0.02; t <= Td_; t += Tm_)
                    Cy.push_back((Ky / Td_) * t);
                for (; t <= T_ - Td_; t += Tm_)
                    Cy.push_back(Cy1 * cosh(wn_ * (t - Td_)) + Cy2 * sinh(wn_ * (t - Td_)) + A);
                for (; t <= T_; t += Tm_)
                    Cy.push_back((Ky / Td_) * (T_ - t));
                x0_ = -xt;
                y0_ = -yt;

                int i=0;
                for(t=0.02; t<=T_; t+=Tm_)
                {
                    yawRot = AngleAxisd(turnSpline.pos(t), Eigen::Vector3d::UnitZ());
                    pitchRot = AngleAxisd(0, Eigen::Vector3d::UnitY());
                    rollRot = AngleAxisd(0, Eigen::Vector3d::UnitX());
                    quat = rollRot*pitchRot*yawRot;
                    swingfoot_mat.set_R(quat.matrix());
                    yawRot = AngleAxisd(turnSpline.pos(T_-t), Eigen::Vector3d::UnitZ());
                    quat = rollRot*pitchRot*yawRot;
                    supportfoot_mat.set_R(quat.matrix());
                    pitchRot = AngleAxisd(deg2rad(15.0), Eigen::Vector3d::UnitY());

                    body_mat.set_p(Vector3d(bodyXSpline.pos(t), Cy[i++], Cz_));
                    body_mat.set_R(pitchRot.toRotationMatrix());
                    supportfoot_mat.set_p(Vector3d(0.0, support_foot * ROBOT->D()/2.0-laterSpline.pos(t)
                        +(support_foot<0?-footYoffset_: footYoffset_), 0.0));
                    swingfoot_mat.set_p(Vector3d(stepSpline.pos(t), -support_foot*ROBOT->D()/2.0+laterSpline.pos(t)
                        +(support_foot<0?footYoffset_:-footYoffset_), riseSpline.pos(t)));
                    if(support_foot<0)
                    {
                        rightfoot_mat = supportfoot_mat;
                        leftfoot_mat = swingfoot_mat;
                    }
                    else
                    {
                        leftfoot_mat = supportfoot_mat;
                        rightfoot_mat = swingfoot_mat;
                    }
                    if (ROBOT->leg_inverse_kinematics(body_mat, leftfoot_mat, degs, true))
                    {   
                        jdegs[ROBOT->get_joint("jlhip3")->jid_] = rad2deg(degs[0]);   
                        jdegs[ROBOT->get_joint("jlhip2")->jid_] = rad2deg(degs[1]);
                        jdegs[ROBOT->get_joint("jlhip1")->jid_] = rad2deg(degs[2]);
                        jdegs[ROBOT->get_joint("jlknee")->jid_] = rad2deg(degs[3]);
                        jdegs[ROBOT->get_joint("jlankle2")->jid_] = rad2deg(degs[4]);
                        jdegs[ROBOT->get_joint("jlankle1")->jid_] = rad2deg(degs[5]);
                    }
                    else
                    {
                        LOG(LOG_WARN) << t << '\t' << "left leg_inverse_kinematics faied!" << endll;
                    }

                    if (ROBOT->leg_inverse_kinematics(body_mat, rightfoot_mat, degs, false))
                    {   
                        jdegs[ROBOT->get_joint("jrhip3")->jid_] = rad2deg(degs[0]);
                        jdegs[ROBOT->get_joint("jrhip2")->jid_] = rad2deg(degs[1]);
                        jdegs[ROBOT->get_joint("jrhip1")->jid_] = rad2deg(degs[2]);
                        jdegs[ROBOT->get_joint("jrknee")->jid_] = rad2deg(degs[3]);
                        jdegs[ROBOT->get_joint("jrankle2")->jid_] = rad2deg(degs[4]);
                        jdegs[ROBOT->get_joint("jrankle1")->jid_] = rad2deg(degs[5]);
                    }
                    else
                    {
                        LOG(LOG_WARN) << t << '\t' << "right leg_inverse_kinematics faied!" << endll;
                    }
                    jdegs[ROBOT->get_joint("jlshoulder1")->jid_] = 40;
                    jdegs[ROBOT->get_joint("jlelbow")->jid_] = -90;
                    jdegs[ROBOT->get_joint("jrshoulder1")->jid_] = 40;
                    jdegs[ROBOT->get_joint("jrelbow")->jid_] = 90;
                    while (!MADT->body_empty())
                    {
                        usleep(500);
                    }
                    if (!MADT->add_body_degs(jdegs))
                    {
                        break;
                    } 
                }
                support_foot *= -1.0;
                WM->navigation(Vector3d(xt, yt, dt));
                if (MADT->get_mode() == adapter::MODE_READY)
                {
                    para_mutex_.lock();
                    enable_ = false;
                    para_mutex_.unlock();
                    if(MADT->get_last_mode() == adapter::MODE_ACT)
                        MADT->set_mode(adapter::MODE_WALK);
                    else
                        MADT->set_mode(adapter::MODE_ACT);
                }
            }
            usleep(500);
        }
    }
}