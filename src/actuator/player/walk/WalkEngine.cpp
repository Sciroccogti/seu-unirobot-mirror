#include "WalkEngine.hpp"
#include "CubicSpline.hpp"
#include "SmoothSpline.hpp"
#include "Polynom.hpp"
#include "robot/humanoid.hpp"
#include "math/math.hpp"
#include "configuration.hpp"
#include <cmath>
#include <fstream>

namespace walk
{
    using namespace Eigen;
    using namespace robot;
    using namespace robot_math;

    WalkEngine::WalkEngine()
    {
        xrange[0] = CONF.get_config_value<double>("walk.x.min");
        xrange[1] = CONF.get_config_value<double>("walk.x.max");
        yrange[0] = CONF.get_config_value<double>("walk.y.min");
        yrange[1] = CONF.get_config_value<double>("walk.y.max");
        drange[0] = CONF.get_config_value<double>("walk.dir.min");
        drange[1] = CONF.get_config_value<double>("walk.dir.max");

        params_.freq = CONF.get_config_value<double>("walk.freq");
        params_.footYOffset = CONF.get_config_value<double>("walk.footYOffset");

        params_.trunkZOffset = CONF.get_config_value<double>("walk.trunkZOffset");
        params_.trunkXOffset = CONF.get_config_value<double>("walk.trunkXOffset");
        params_.trunkPitch = deg2rad(CONF.get_config_value<double>("walk.trunkPitch"));
        params_.trunkYOffset = 0.0;
        params_.trunkRoll = 0.0;

        params_.swingGain = CONF.get_config_value<float>("walk.swingGain");
        params_.swingPhase = CONF.get_config_value<float>("walk.swingPhase");
        params_.swingRollGain = 0.0;

        params_.supportPhaseRatio = 0.0;
        params_.stepUpVel = 4.0;
        params_.stepDownVel = 4.0;
        params_.riseUpVel = 4.0;
        params_.riseDownVel = 0.0;
        params_.swingPause = 0.0;
        params_.swingVel = 4.0;

        params_.extraLeftX = 0.0;
        params_.extraLeftY = 0.0;
        params_.extraLeftZ = 0.0;
        params_.extraRightX = 0.0;
        params_.extraRightY = 0.0;
        params_.extraRightZ = 0.0;
        params_.extraLeftYaw = 0.0;
        params_.extraLeftPitch = 0.0;
        params_.extraLeftRoll = 0.0;
        params_.extraRightYaw = 0.0;
        params_.extraRightPitch = 0.0;
        params_.extraRightRoll = 0.0;

        //The walk is started while walking on place
        params_.enabledGain = 1.0;
        params_.stepGain = 0.0;
        params_.lateralGain = 0.0;
        params_.turnGain = 0.0;

        phase_ = 0.0;
        dt_ = 0.0;
        support_foot_ = DOUBLE_SUPPORT;
        enable_ = false;
    }

    WalkEngine::~WalkEngine()
    {
        if(td_.joinable()) td_.join();
    }

    void WalkEngine::start(sensor_ptr s)
    {
        motor_ = std::dynamic_pointer_cast<motor>(s);
        dt_ = 1.0/motor_->freq();
        is_alive_ = true;
        td_ = std::move(std::thread(&WalkEngine::run, this));
    }

    void WalkEngine::boundPhase(double &phase)
    {
        while (phase >= 1.0)
        {
            phase -= 1.0;
            if (phase < 0.0) phase = 0.0;
        }
    }

    void WalkEngine::set_params(const Eigen::Vector4f &params)
    {
        para_mutex_.lock();
        params_.stepGain = params[0];
        params_.lateralGain = params[1];
        params_.turnGain = params[2];
        params_.riseGain = params[3];
        bound(xrange[0], xrange[1], params_.stepGain);
        bound(yrange[0], yrange[1], params_.lateralGain);
        bound(drange[0], drange[1], params_.turnGain);
        bound(0.0, 0.04, params_.riseGain);
        params_.turnGain = deg2rad(params_.turnGain);
        para_mutex_.unlock();
    }

    void WalkEngine::run()
    {
        double stepLength = 0.5 * params_.supportPhaseRatio + 0.5;
        CubicSpline stepSpline;
        stepSpline.addPoint(0.0, 0.5, -1.0 / stepLength);
        stepSpline.addPoint(stepLength, -0.5, -1.0 / stepLength);
        stepSpline.addPoint(stepLength, -0.5, params_.stepUpVel);
        stepSpline.addPoint(1.0, 0.5, -params_.stepDownVel);

        CubicSpline swingSpline;
        swingSpline.addPoint(0.0, -1.0);
        swingSpline.addPoint(params_.swingPause / 2.0, -1.0);
        swingSpline.addPoint(params_.swingPause / 2.0, -1.0, params_.swingVel);
        swingSpline.addPoint(0.5 - params_.swingPause / 2.0, 1.0, params_.swingVel);
        swingSpline.addPoint(0.5 - params_.swingPause / 2.0, 1.0);
        swingSpline.addPoint(0.5 + params_.swingPause / 2.0, 1.0);
        swingSpline.addPoint(0.5 + params_.swingPause / 2.0, 1.0, -params_.swingVel);
        swingSpline.addPoint(1.0 - params_.swingPause / 2.0, -1.0, -params_.swingVel);
        swingSpline.addPoint(1.0 - params_.swingPause / 2.0, -1.0);
        swingSpline.addPoint(1.0, -1.0, 0.0);

        CubicSpline riseSpline;
        riseSpline.addPoint(0.0, 0.0);
        riseSpline.addPoint(stepLength, 0.0);
        riseSpline.addPoint(stepLength, 0.0, params_.riseUpVel);
        riseSpline.addPoint((1.0 + stepLength) / 2.0, 1.0);
        riseSpline.addPoint(1.0, 0.0, -params_.riseDownVel);

        SmoothSpline turnSpline;
        turnSpline.addPoint(0.0, 0.0);
        turnSpline.addPoint(stepLength - 0.5, 0.0);
        turnSpline.addPoint(0.5, 1.0);
        turnSpline.addPoint(stepLength, 1.0);
        turnSpline.addPoint(1.0, 0.0);

        transform_matrix body_mat, leftfoot_mat, rightfoot_mat;
        Quaternion<double> quat;
        AngleAxisd yawRot, pitchRot, rollRot;
        std::vector<double> degs;
        std::map<int, float> jdegs;
        bool e, ra;
        float tt=0.0;
        while(is_alive_)
        {
            double phaseLeft = phase_;
            double phaseRight = phase_ + 0.5;
            boundPhase(phaseLeft);
            boundPhase(phaseRight);
            if(phaseLeft<0.5)
                support_foot_ = LEFT_SUPPORT;
            else
                support_foot_ = RIGHT_SUPPORT;

            para_mutex_.lock();
            //Compute swing value
            double swingVal = params_.enabledGain * params_.swingGain
                              * swingSpline.posMod(0.5 + phaseLeft + params_.swingPhase);

            //Compute feet forward (step) oscillation
            double leftX = params_.enabledGain * params_.stepGain * stepSpline.pos(phaseLeft);
            double rightX = params_.enabledGain * params_.stepGain * stepSpline.pos(phaseRight);

            //Compute feet swing oscillation
            double leftY = swingVal + ROBOT.D()/2.0;
            double rightY = swingVal - ROBOT.D()/2.0;
            //Compute feet lateral movement oscillation
            leftY += params_.enabledGain * params_.lateralGain * (stepSpline.pos(phaseLeft)
                    + 0.5 * (params_.lateralGain >= 0.0 ? 1.0 : -1.0));
            rightY += params_.enabledGain * params_.lateralGain * (stepSpline.pos(phaseRight)
                    + 0.5 * (params_.lateralGain >= 0.0 ? -1.0 : 1.0));
            leftY += params_.footYOffset;
            rightY += -params_.footYOffset;

            //Compute feet vertical (rise) oscillation and offset
            double leftZ = params_.enabledGain * params_.riseGain * riseSpline.pos(phaseLeft);
            double rightZ = params_.enabledGain * params_.riseGain * riseSpline.pos(phaseRight);
            leftZ += params_.trunkZOffset;
            rightZ += params_.trunkZOffset;

            //Compute feet rotation (turn) oscillation
            double leftYaw = params_.enabledGain * params_.turnGain * turnSpline.pos(phaseLeft);
            double rightYaw = params_.enabledGain * params_.turnGain * turnSpline.pos(phaseRight);
            para_mutex_.unlock();

            //Compute trunk roll angle
            double rollVal = params_.enabledGain * -params_.swingRollGain
                             * swingSpline.posMod(0.5 + phaseLeft + params_.swingPhase);
            //Set trunk roll offset
            rollVal += params_.trunkRoll;

            //Set feet orientation
            double leftPitch = 0;
            double leftRoll = rollVal;
            double rightPitch = 0;
            double rightRoll = rollVal;

            //Add custom extra foot offset on both feet
            leftX += params_.extraLeftX;
            leftY += params_.extraLeftY;
            leftZ += params_.extraLeftZ;
            leftYaw += params_.extraLeftYaw;
            leftPitch += params_.extraLeftPitch;
            leftRoll += params_.extraLeftRoll;
            rightX += params_.extraRightX;
            rightY += params_.extraRightY;
            rightZ += params_.extraRightZ;
            rightYaw += params_.extraRightYaw;
            rightPitch += params_.extraRightPitch;
            rightRoll += params_.extraRightRoll;

            //Build rotation matrix for trunk pitch and roll
            //orientation
            //pitchRot = AngleAxisd(-params_.trunkPitch, Vector3d::UnitY());
            rollRot = AngleAxisd(-rollVal, Vector3d::UnitX());
            quat = Quaternion<double>(rollRot);
            Matrix3d rotation = quat.matrix();

            //Build target vector.
            //Used Euler angles orders is Pitch Roll Yaw because
            //Yaw has to be applied last, after the foot get the good
            //ground orientation. Roll has to be applied after Pitch.
            Vector3d posLeft(leftX, leftY, leftZ);
            Vector3d angleLeft(leftPitch, leftRoll, leftYaw);
            Vector3d posRight(rightX, rightY, rightZ);
            Vector3d angleRight(rightPitch, rightRoll, rightYaw);

            //Rotate built feet trajectory to
            //meet asked trunk Pitch and Roll new
            //ground orientation
            posLeft = rotation*posLeft;
            posRight = rotation*posRight;

            //Apply trunk X-Y offset
            posLeft(0) -= params_.trunkXOffset;
            posRight(0) -= params_.trunkXOffset;
            posLeft(1) -= params_.trunkYOffset;
            posRight(1) -= params_.trunkYOffset;

            //In case of trunk Roll rotation, an height (Z)
            //positive offset have to be applied on external foot to
            //set both feet on same level
            double deltaLen = ROBOT.D()*tan(rollVal);
            if (rollVal > 0.0)
                posRight(2) += deltaLen;
            else if (rollVal < 0.0)
                posLeft(2) -= deltaLen;

            //Trunk X and Y offset is applied to compensate
            //Pitch and Roll rotation. It is better for tunning if
            //trunk pitch or roll rotation do not apply offset on
            //trunk position.
            //posLeft(0) += (ROBOT.leg_length())*tan(params_.trunkPitch);
            //posRight(0) += (ROBOT.leg_length())*tan(params_.trunkPitch);
            posLeft(1) -= (ROBOT.leg_length())*tan(rollVal);
            posRight(1) -= (ROBOT.leg_length())*tan(rollVal);

            body_mat.set_p(Vector3d(0, 0, ROBOT.leg_length()));
            leftfoot_mat.set_p(posLeft);
            rightfoot_mat.set_p(posRight);

            pitchRot = AngleAxisd(params_.trunkPitch, Vector3d::UnitY());
            quat = Quaternion<double>(pitchRot);
            body_mat.set_R(quat.matrix());
            yawRot = AngleAxisd(angleLeft[2], Vector3d::UnitZ());
            pitchRot = AngleAxisd(angleLeft[1], Vector3d::UnitY());
            rollRot = AngleAxisd(angleLeft[0], Vector3d::UnitX());
            quat = Quaternion<double>(rollRot * pitchRot * yawRot);
            leftfoot_mat.set_R(quat.matrix());
            yawRot = AngleAxisd(angleRight[2], Vector3d::UnitZ());
            pitchRot = AngleAxisd(angleRight[1], Vector3d::UnitY());
            rollRot = AngleAxisd(angleRight[0], Vector3d::UnitX());
            quat = Quaternion<double>(rollRot * pitchRot * yawRot);
            rightfoot_mat.set_R(quat.matrix());

            if (ROBOT.leg_inverse_kinematics(body_mat, leftfoot_mat, degs, true))
            {
                jdegs[ROBOT.get_joint("jlhip3")->jid_] = rad2deg(degs[0]);
                jdegs[ROBOT.get_joint("jlhip2")->jid_] = rad2deg(degs[1]);
                jdegs[ROBOT.get_joint("jlhip1")->jid_] = rad2deg(degs[2]);
                jdegs[ROBOT.get_joint("jlknee")->jid_] = rad2deg(degs[3]);
                jdegs[ROBOT.get_joint("jlankle2")->jid_] = rad2deg(degs[4]);
                jdegs[ROBOT.get_joint("jlankle1")->jid_] = rad2deg(degs[5]);
                if(support_foot_ == LEFT_SUPPORT) ROBOT.leg_forward_kinematics(degs, true);
            }
            else std::cout<<phase_<<'\t'<<"\033[31mleft leg_inverse_kinematics faied!\033[0m"<<std::endl;
            if (ROBOT.leg_inverse_kinematics(body_mat, rightfoot_mat, degs, false))
            {
                jdegs[ROBOT.get_joint("jrhip3")->jid_] = rad2deg(degs[0]);
                jdegs[ROBOT.get_joint("jrhip2")->jid_] = rad2deg(degs[1]);
                jdegs[ROBOT.get_joint("jrhip1")->jid_] = rad2deg(degs[2]);
                jdegs[ROBOT.get_joint("jrknee")->jid_] = rad2deg(degs[3]);
                jdegs[ROBOT.get_joint("jrankle2")->jid_] = rad2deg(degs[4]);
                jdegs[ROBOT.get_joint("jrankle1")->jid_] = rad2deg(degs[5]);
                if(support_foot_ == RIGHT_SUPPORT) ROBOT.leg_forward_kinematics(degs, false);
            }
            else std::cout<<phase_<<'\t'<<"\033[31mright leg_inverse_kinematics faied!\033[0m"<<std::endl;

            jdegs[ROBOT.get_joint("jlshoulder1")->jid_] = 20.0;
            jdegs[ROBOT.get_joint("jlelbow")->jid_] = -160.0;
            jdegs[ROBOT.get_joint("jrshoulder1")->jid_] = 20.0;
            jdegs[ROBOT.get_joint("jrelbow")->jid_] = 160.0;
            e_mutex_.lock();
            e = enable_;
            e_mutex_.unlock();
            if(e)
            {
                while(!motor_->body_empty());
                if(!motor_->add_body_degs(jdegs)) break;
            }
            phase_ += dt_*params_.freq;
            boundPhase(phase_);
        }
    }
}