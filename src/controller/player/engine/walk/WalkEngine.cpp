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

namespace motion
{
    using namespace Eigen;
    using namespace robot;
    using namespace robot_math;

    WalkEngine::WalkEngine()
    {
        std::vector<double> range = CONF->get_config_vector<double>("walk.x");
        xrange[0] = range[0];
        xrange[1] = range[1];
        range = CONF->get_config_vector<double>("walk.y");
        yrange[0] = range[0];
        yrange[1] = range[1];
        range = CONF->get_config_vector<double>("walk.dir");
        drange[0] = range[0];
        drange[1] = range[1];

        params_.freq = CONF->get_config_value<double>("walk.freq");
        params_.riseGain = CONF->get_config_value<double>("walk.rise");

        params_.footYOffset = CONF->get_config_value<double>("walk.footYOffset");
        params_.trunkZOffset = CONF->get_config_value<double>("walk.trunkZOffset");
        params_.trunkXOffset = CONF->get_config_value<double>("walk.trunkXOffset");
        params_.trunkYOffset = CONF->get_config_value<double>("walk.trunkYOffset");

        params_.trunkPitch = deg2rad(CONF->get_config_value<double>("walk.trunkPitch"));

        
        params_.trunkRoll = 0.0;

        params_.supportPhaseRatio = CONF->get_config_value<double>("walk.doubleSupportRatio");
        params_.swingGain = CONF->get_config_value<double>("walk.swingGain");
        params_.swingPhase = CONF->get_config_value<double>("walk.swingPhase");
        params_.swingRollGain = 0.0;

        params_.stepUpVel = 4.5; 
        params_.stepDownVel = 4.5;
        params_.riseUpVel = 4.4;
        params_.riseDownVel = 0.0;
        params_.swingPause = 0.1;
        params_.swingVel = 4.0;

        params_.extraLeftX = CONF->get_config_value<double>("walk.extraLeftX");  
        params_.extraLeftY = CONF->get_config_value<double>("walk.extraLeftY");                                                   //0.002;
        params_.extraLeftZ = CONF->get_config_value<double>("walk.extraLeftZ");
        params_.extraRightX = CONF->get_config_value<double>("walk.extraRightX");
        params_.extraRightY = CONF->get_config_value<double>("walk.extraRightY");
        params_.extraRightZ = CONF->get_config_value<double>("walk.extraRightZ");
        params_.extraLeftYaw = 0.0;
        params_.extraLeftPitch = 0.0;
        params_.extraLeftRoll = 0.0;
        params_.extraRightYaw = 0.0;
        params_.extraRightPitch = 0.0;
        params_.extraRightRoll = 0.0;

        //for walking step
        params_.stepKxr = CONF->get_config_value<double>("walk.stepKxr");
        params_.stepKxl = CONF->get_config_value<double>("walk.stepKxl");
        params_.stepKyr = CONF->get_config_value<double>("walk.stepKyr");
        params_.stepKyl = CONF->get_config_value<double>("walk.stepKyl");
        params_.stepKzr = CONF->get_config_value<double>("walk.stepKzr");
        params_.stepKzl = CONF->get_config_value<double>("walk.stepKzl");

        //intial for feedback parameters
        params_.feedStepKxl = 1.0;
        params_.feedStepKxr = 1.0;
        params_.feedStepKyl = 1.0;
        params_.feedStepKyr = 1.0;
        params_.feedStepKzl = 1.0;
        params_.feedStepKzr = 1.0;


        //The walk is started while walking on place
        params_.enabledGain = 0.0;
        params_.stepGain = 0.0;
        params_.lateralGain = 0.0;
        params_.turnGain = 0.0;

        phase_ = 0.0;
        dt_ = 0.0;
    }

    void WalkEngine::updata(const pub_ptr &pub, const int &type)
    {
        if (type == sensor::SENSOR_IMU)
        {
            imu_mtx_.lock();
            std::shared_ptr<imu> sptr = std::dynamic_pointer_cast<imu>(pub);
            imu_data_ = sptr->data();
            imu_mtx_.unlock();
            return;
        }

        if (type == sensor::SENSOR_MOTOR)
        {
            dxl_mtx_.lock();
            std::shared_ptr<motor> sptr = std::dynamic_pointer_cast<motor>(pub);
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
        LOG << std::setw(12) << "engine:" << std::setw(18) << "[WalkEngine]" << " ended!" << ENDL;
    }

    void WalkEngine::start()
    {
        dt_ = 1.0 / (1000.0 / CONF->get_config_value<double>("hardware.motor.period"));
        is_alive_ = true;
        td_ = std::move(std::thread(&WalkEngine::run, this));
        LOG << std::setw(12) << "engine:" << std::setw(18) << "[WalkEngine]" << " started!" << ENDL;
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

    void WalkEngine::switchPhaseFlag()
    {
        if(phase_flag == false){
            phase_flag = true;
        }else{
            phase_flag = false;
        }
        
    }

    void WalkEngine::set_params(float x, float y, float d, bool enable)
    {
        para_mutex_.lock();
        params_.stepGain = x;
        params_.lateralGain = y;
        params_.turnGain = d;
        params_.enabledGain = enable?1.0:0.0;
        bound(xrange[0], xrange[1], params_.stepGain);
        bound(yrange[0], yrange[1], params_.lateralGain);
        bound(drange[0], drange[1], params_.turnGain);
        params_.turnGain = deg2rad(params_.turnGain);
        para_mutex_.unlock();
    }

    void WalkEngine::run()
    {   
        double len = 0.5 * params_.supportPhaseRatio;    //0.5*0.2 = 0.1 pause length in half step
        CubicSpline stepSpline;
        double V_y0 = (0.5 - len)/(1 + 2*len);
        stepSpline.addPoint(0.0,       V_y0,  -1.0 / (len + 0.5));
        stepSpline.addPoint(0.5,       -0.5,  -1.0 / (len + 0.5));
        stepSpline.addPoint(0.5,       -0.5,  params_.stepUpVel);
        stepSpline.addPoint(1.0 - len,  0.5,  -params_.stepDownVel);
        stepSpline.addPoint(1.0 - len,  0.5,  -1.0 / (len + 0.5));
        stepSpline.addPoint(1.0,       V_y0,  -1.0 / (len + 0.5)); 

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
        riseSpline.addPoint(0.5, 0.0);
        riseSpline.addPoint(0.5,              0.0, params_.riseUpVel);
        riseSpline.addPoint(0.75 - 0.5*len,   1.0);
        riseSpline.addPoint(1.0 - len, 0.0, -params_.riseDownVel);
        riseSpline.addPoint(1.0 - len, 0.0);
        riseSpline.addPoint(1.0,       0.0);  

        SmoothSpline turnSpline;
        turnSpline.addPoint(0.0, 0.0);
        turnSpline.addPoint(0.5 - len, 1.0);
        turnSpline.addPoint(0.5,       1.0);
        turnSpline.addPoint(1.0 - len, 0.0);
        turnSpline.addPoint(1.0, 0.0);

        SmoothSpline handSpline;
        handSpline.addPoint(0.0, 0.0);
        handSpline.addPoint(0.25, 1.0);
        handSpline.addPoint(0.5, 0.0);
        handSpline.addPoint(0.75, -1.0);
        handSpline.addPoint(1.0, 0.0);

        transform_matrix body_mat, leftfoot_mat, rightfoot_mat;
        Quaternion<double> quat;
        AngleAxisd yawRot, pitchRot, rollRot;
        Vector3d lefthand, righthand;
        std::vector<double> degs;
        std::map<int, float> jdegs;
        double phaseLeft, phaseRight;
        double handGain = 0.1;
        WalkParameters tempParams;

        phase_flag = false;
        feedbackWalkComputor FWC;
        
        int recode = 0;


        while (is_alive_)
        {   
            phase_ = 0.0;
            switchPhaseFlag();
            jdegs.clear();
            para_mutex_.lock();
            tempParams = params_;
            para_mutex_.unlock();

            if (MADT->get_mode() == adapter::MODE_READY)
            {
                tempParams.stepGain = 0.0;
                tempParams.lateralGain = 0.0;
                tempParams.turnGain = 0.0;
            }
            if((fabs(tempParams.enabledGain)>1E-2) &&(MADT->get_mode() == adapter::MODE_READY || MADT->get_mode() == adapter::MODE_WALK))
            {                   
                while (phase_ < 0.5)
                {   
                    phaseLeft = (phase_flag==true ? phase_ : phase_+0.5);
                    phaseRight = phaseLeft + 0.5;
                    boundPhase(phaseLeft);
                    boundPhase(phaseRight);

                    if (phaseLeft < 0.5)
                    {
                        WM->set_support_foot(LEFT_SUPPORT);
                    }
                    else
                    {
                        WM->set_support_foot(RIGHT_SUPPORT);
                    }

                    //Compute swing value
                    double swingVal = tempParams.enabledGain * tempParams.swingGain
                                      * swingSpline.posMod(0.5 + phaseLeft + tempParams.swingPhase + len);

                    //Compute feet forward (step) oscillation
                    double leftX = tempParams.enabledGain * tempParams.stepGain * stepSpline.pos(phaseLeft) * tempParams.stepKxl;
                    double rightX = tempParams.enabledGain * tempParams.stepGain * stepSpline.pos(phaseRight) * tempParams.stepKxr;

                    //Compute feet swing oscillation
                    double leftY = swingVal;       
                    double rightY = swingVal;      
                    //Compute feet lateral movement oscillation
                    leftY += tempParams.enabledGain * tempParams.lateralGain * (stepSpline.pos(phaseLeft)
                             + 0.5 * (tempParams.lateralGain >= 0.0 ? 1.0 : -1.0));
                    rightY += tempParams.enabledGain * tempParams.lateralGain * (stepSpline.pos(phaseRight)
                              + 0.5 * (tempParams.lateralGain >= 0.0 ? -1.0 : 1.0));
                    leftY += tempParams.footYOffset;
                    rightY += -tempParams.footYOffset;

                    //Compute feet vertical (rise) oscillation and offset
                    double leftZ = tempParams.enabledGain * tempParams.riseGain * riseSpline.pos(phaseLeft) * tempParams.stepKzl;
                    double rightZ = tempParams.enabledGain * tempParams.riseGain * riseSpline.pos(phaseRight) * tempParams.stepKzr;
                    leftZ += tempParams.trunkZOffset;
                    rightZ += tempParams.trunkZOffset;

                    //Compute feet rotation (turn) oscillation
                    double leftYaw = tempParams.enabledGain * tempParams.turnGain * turnSpline.pos(phaseLeft);
                    double rightYaw = tempParams.enabledGain * tempParams.turnGain * turnSpline.pos(phaseRight);

                    //Compute trunk roll angle
                    double rollVal = tempParams.enabledGain * -tempParams.swingRollGain
                                     * swingSpline.posMod(0.5 + phaseLeft + tempParams.swingPhase);
                    //Set trunk roll offset
                    rollVal += tempParams.trunkRoll;

                    //Set feet orientation 
                    double leftPitch = 0;
                    double leftRoll = rollVal;
                    double rightPitch = 0;
                    double rightRoll = rollVal;

                    //Add custom extra foot offset on both feet
                    leftX += tempParams.extraLeftX;
                    leftY += tempParams.extraLeftY;
                    leftZ += tempParams.extraLeftZ;
                    leftYaw += tempParams.extraLeftYaw;
                    leftPitch += tempParams.extraLeftPitch;
                    leftRoll += tempParams.extraLeftRoll;
                    rightX += tempParams.extraRightX;
                    rightY += tempParams.extraRightY;
                    rightZ += tempParams.extraRightZ;
                    rightYaw += tempParams.extraRightYaw;
                    rightPitch += tempParams.extraRightPitch;
                    rightRoll += tempParams.extraRightRoll;

                    //Build rotation matrix for trunk pitch and roll
                    //orientation
                    //pitchRot = AngleAxisd(-tempParams.trunkPitch, Vector3d::UnitY());
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
                    posLeft = rotation * posLeft;
                    posRight = rotation * posRight;

                    //Apply trunk X-Y offset
                    posLeft(0) -= tempParams.trunkXOffset;
                    posRight(0) -= tempParams.trunkXOffset;
                    posLeft(1) -= tempParams.trunkYOffset;
                    posRight(1) -= tempParams.trunkYOffset;

                    //In case of trunk Roll rotation, an height (Z)
                    //positive offset have to be applied on external foot to
                    //set both feet on same level
                    double deltaLen = ROBOT->D() * tan(rollVal);

                    if (rollVal > 0.0)
                    {
                        posRight(2) += deltaLen;
                    }
                    else if (rollVal < 0.0)
                    {
                        posLeft(2) -= deltaLen;
                    }

                    //Trunk X and Y offset is applied to compensate
                    //Pitch and Roll rotation. It is better for tunning if
                    //trunk pitch or roll rotation do not apply offset on
                    //trunk position.
                    posLeft(0) += 0 ;
                    posRight(0) += 0 ;
                    posLeft(1) -= (ROBOT->leg_length_without_foot()) * tan(rollVal);
                    posRight(1) -= (ROBOT->leg_length_without_foot()) * tan(rollVal);

                    posLeft(1) += ROBOT->D() / 2.0;
                    posRight(1) -= ROBOT->D() / 2.0;

                    body_mat.set_p(Vector3d(0, 0, ROBOT->leg_length_without_foot()));
                    leftfoot_mat.set_p(posLeft);
                    rightfoot_mat.set_p(posRight);

                    pitchRot = AngleAxisd(tempParams.trunkPitch, Vector3d::UnitY());
                    quat = Quaternion<double>(pitchRot);
                    body_mat.set_R(quat.matrix());
                    
                    //PitchRollYaw
                    yawRot = AngleAxisd(angleLeft[2], Vector3d::UnitZ());
                    pitchRot = AngleAxisd(angleLeft[0], Vector3d::UnitY());
                    rollRot = AngleAxisd(angleLeft[1], Vector3d::UnitX());
                    quat = Quaternion<double>(yawRot * rollRot * pitchRot);    //rollRot * pitchRot * yawRot
                    leftfoot_mat.set_R(quat.matrix());

                    yawRot = AngleAxisd(angleRight[2], Vector3d::UnitZ());
                    pitchRot = AngleAxisd(angleRight[0], Vector3d::UnitY());
                    rollRot = AngleAxisd(angleRight[1], Vector3d::UnitX());
                    quat = Quaternion<double>(yawRot * rollRot * pitchRot);
                    rightfoot_mat.set_R(quat.matrix());
                    
                    // double keenl,keenr;
                    ROBOT->body_mat = body_mat;
                    if (ROBOT->leg_inverse_kinematics_walk(body_mat, leftfoot_mat, degs, true))
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
                        LOG << phase_ << '\t' << "left leg_inverse_kinematics faied!" << ENDL;
                    }

                    if (ROBOT->leg_inverse_kinematics_walk(body_mat, rightfoot_mat, degs, false))
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
                        LOG << phase_ << '\t' << "right leg_inverse_kinematics faied!" << ENDL;
                    }

                    jdegs[ROBOT->get_joint("jlshoulder1")->jid_] = 49.43;
                    jdegs[ROBOT->get_joint("jlelbow")->jid_] = -119.24;
                    jdegs[ROBOT->get_joint("jrshoulder1")->jid_] = 49.43;
                    jdegs[ROBOT->get_joint("jrelbow")->jid_] = 119.24;
        

                    if((phase_+ dt_ * tempParams.freq) >= 0.5 && tempParams.enabledGain == 1.0)
                    {   
                        /*
                        * when jdegs[0] = 1.0, rightfoot is swingfoot.
                        * when jdegs[0] = 2.0, lefttfoot is swingfoot.
                        * this means this is the last frame of one half step
                        */
                        jdegs[0] = phaseLeft < 0.5? 1.0 : 2.0;                     
                    }
                    while (!MADT->body_empty())
                    {
                        usleep(500);
                    }
                    if (!MADT->add_body_degs(jdegs))
                    {
                        break;
                    } 
                    phase_ += dt_ * tempParams.freq;  
                }

                WM->navigation(Vector3d(tempParams.stepGain, tempParams.lateralGain, tempParams.turnGain));
                //start the inverse computation of  the feedback loop for walk engine
                if(tempParams.enabledGain == 1.0)
                {   
                    while(ROBOT->conveyFeedbackParams.update_flag == false && is_alive_)
                    {
                       //wait for the motor consume the whole step; 
                       usleep(500);
                    } 
                    //std::cout<<"calculate the feed walk params. ======="<<std::endl;
                    /*use the conveyFeedbackParams to update feedback parameters in the place*/

                    ROBOT->conveyFeedbackParams.update_flag = false;  
                }

                if (MADT->get_mode() == adapter::MODE_READY)
                {
                    para_mutex_.lock();
                    params_.enabledGain = 0.0;
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