#include "WalkEngine.hpp"
#include "CubicSpline.hpp"
#include "Polynom.hpp"
#include "robot/humanoid.hpp"
#include "math/math.hpp"
#include <cmath>
#include <fstream>

namespace walk
{
    using namespace Eigen;
    using namespace robot;
    using namespace robot_math;

    WalkEngine::WalkEngine()
    {
        x0=0.0;
        y0=0.0;
        Cz=ROBOT.leg_length()-0.02;
        wn=sqrt(9.8/Cz);
        supportfoot = -1.0;
    }

    WalkEngine::~WalkEngine()
    {
        if(td_.joinable()) td_.join();
    }

    void WalkEngine::start(sensor_ptr s)
    {
        motor_ = std::dynamic_pointer_cast<motor>(s);
        is_alive_ = true;
        td_ = std::move(std::thread(&WalkEngine::run, this));
    }

    void WalkEngine::set_params(const Eigen::Vector4f &params)
    {
        para_mutex_.lock();
        xt = params[0];
        yt = params[1];
        dt = params[2];
        ht = params[3];
        bound(-x_range, x_range, xt);
        bound(-y_range, y_range, yt);
        bound(-d_range, d_range, dt);
        bound(0.0f, h_max, ht);
        para_mutex_.unlock();
    }

    void WalkEngine::run()
    {
        transform_matrix body_mat, supportfoot_mat, swingfoot_mat, leftfoot_mat, rightfoot_mat;
        Quaternion<double> quat;
        AngleAxisd yawRot, pitchRot, rollRot;
        std::vector<double> degs;
        std::map<int, float> jdegs;
        bool e;
        float tt=0.0;
        while(is_alive_)
        {
            e_mutex_.lock();
            e = enable_;
            e_mutex_.unlock();
            if(e)
            {
                para_mutex_.lock();
                //std::cout<<"xt: "<<xt<<std::endl;
                CubicSpline stepSpline;
                stepSpline.addPoint(0.0, x0);
                stepSpline.addPoint(td, x0);
                stepSpline.addPoint(T-td, xt);
                stepSpline.addPoint(T, xt);

                CubicSpline laterSpline;
                laterSpline.addPoint(0.0, y0);
                laterSpline.addPoint(td, y0);
                laterSpline.addPoint(T-td, yt);
                laterSpline.addPoint(T, yt);

                CubicSpline riseSpline;
                riseSpline.addPoint(0.0, 0.0);
                riseSpline.addPoint(td, 0.0);
                riseSpline.addPoint(T / 2.0, ht);
                riseSpline.addPoint(T - td, 0.0);
                riseSpline.addPoint(T, 0.0);

                CubicSpline turnSpline;
                turnSpline.addPoint(0.0, 0.0);
                turnSpline.addPoint(td, 0.0);
                turnSpline.addPoint(T - td, deg2rad(dt));
                turnSpline.addPoint(T, deg2rad(dt));

                float B = (xt - x0) / 2.0, A = supportfoot * ROBOT.D()/2.0;
                float Kx = B * td * wn / (td * wn + tanh(wn * (T / 2 - td)));
                CubicSpline bodyXSpline;
                bodyXSpline.addPoint(0.0, x0/2.0, Kx/td);
                bodyXSpline.addPoint(T, xt/2.0, Kx/td);

                float Ky = A * td * wn * tanh(wn * (T / 2 - td)) / (1 + td * wn * tanh(wn * (T / 2 - td)));
                float Cy1 = Ky - A, Cy2 = Ky / (td * wn);
                float t;
                std::map<float, float> Cy;
                Cy.clear();
                for (t = 0.01; t <= td; t += 0.01)
                    Cy[t] = (Ky / td) * t;
                for (; t <= T - td; t += 0.01)
                    Cy[t] = Cy1 * cosh(wn * (t - td)) + Cy2 * sinh(wn * (t - td)) + A;
                for (; t <= T; t += 0.01)
                    Cy[t] = (Ky / td) * (T - t);
                x0 = -xt;
                y0 = -yt;
                para_mutex_.unlock();

                //std::ofstream out("line.txt", std::ios::app);
                for (t = 0.01; t <= T; t += 0.01)
                {
                    //tt+=0.01;
                    //out<<tt<<'\t'<<bodyXSpline.pos(t)<<std::endl;
                    yawRot = AngleAxisd(turnSpline.pos(t), Eigen::Vector3d::UnitZ());
                    pitchRot = AngleAxisd(0, Eigen::Vector3d::UnitY());
                    rollRot = AngleAxisd(0, Eigen::Vector3d::UnitX());
                    quat = rollRot*pitchRot*yawRot;
                    swingfoot_mat.set_R(quat.matrix());
                    yawRot = AngleAxisd(turnSpline.pos(T-t), Eigen::Vector3d::UnitZ());
                    quat = rollRot*pitchRot*yawRot;
                    supportfoot_mat.set_R(quat.matrix());

                    body_mat.set_p(Vector3d(bodyXSpline.pos(t), Cy[t], Cz));
                    supportfoot_mat.set_p(Vector3d(0.0, supportfoot * ROBOT.D()/2.0-laterSpline.pos(t)
                        +(supportfoot<0?-footYoffset: footYoffset), 0.0));
                    swingfoot_mat.set_p(Vector3d(stepSpline.pos(t), -supportfoot*ROBOT.D()/2.0+laterSpline.pos(t)
                        +(supportfoot<0?footYoffset:-footYoffset), riseSpline.pos(t)));
                    if(supportfoot<0)
                    {
                        rightfoot_mat = supportfoot_mat;
                        leftfoot_mat = swingfoot_mat;
                    }
                    else
                    {
                        leftfoot_mat = supportfoot_mat;
                        rightfoot_mat = swingfoot_mat;
                    }
                    if (ROBOT.leg_inverse_kinematics(body_mat, leftfoot_mat, degs, true))
                    {
                        jdegs[ROBOT.get_joint("jlhip3")->jid_] = rad2deg(degs[0]);
                        jdegs[ROBOT.get_joint("jlhip2")->jid_] = rad2deg(degs[1]);
                        jdegs[ROBOT.get_joint("jlhip1")->jid_] = rad2deg(degs[2]);
                        jdegs[ROBOT.get_joint("jlknee")->jid_] = rad2deg(degs[3]);
                        jdegs[ROBOT.get_joint("jlankle2")->jid_] = rad2deg(degs[4]);
                        jdegs[ROBOT.get_joint("jlankle1")->jid_] = rad2deg(degs[5]);
                    }

                    if (ROBOT.leg_inverse_kinematics(body_mat, rightfoot_mat, degs, false))
                    {
                        jdegs[ROBOT.get_joint("jrhip3")->jid_] = rad2deg(degs[0]);
                        jdegs[ROBOT.get_joint("jrhip2")->jid_] = rad2deg(degs[1]);
                        jdegs[ROBOT.get_joint("jrhip1")->jid_] = rad2deg(degs[2]);
                        jdegs[ROBOT.get_joint("jrknee")->jid_] = rad2deg(degs[3]);
                        jdegs[ROBOT.get_joint("jrankle2")->jid_] = rad2deg(degs[4]);
                        jdegs[ROBOT.get_joint("jrankle1")->jid_] = rad2deg(degs[5]);
                    }
                    while(!motor_->body_empty());
                    if(!motor_->add_body_degs(jdegs)) break;
                }
                //out.close();
                supportfoot *= -1.0;
            }
        }
    }
}