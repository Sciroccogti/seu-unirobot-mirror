#ifndef SEU_UNIROBOT_ACTUATOR_WALK_HPP
#define SEU_UNIROBOT_ACTUATOR_WALK_HPP

#include <map>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include "motor/motor.hpp"
#include "singleton.hpp"

namespace walk
{
    struct WalkParameters
    {
        /**
         * Complete (two legs) walk cycle frequency
         * in Hertz
         */
        double freq;
        /**
         * Global gain multiplying all time
         * dependant movement between 0 and 1.
         * Control walk enabled/disabled smoothing.
         * 0 is walk disabled.
         * 1 is walk fully enabled
         */
        double enabledGain;
        /**
         * Length of double support phase
         * in phase time
         * (between 0 and 1)
         * 0 is null double support and full single support
         * 1 is full double support and null single support
         */
        double supportPhaseRatio;
        /**
         * Lateral offset on default foot
         * position in meters (foot lateral distance)
         * 0 is default
         * > 0 is both feet external offset
         */
        double footYOffset;
        /**
         * Forward length of each foot step
         * in meters
         * >0 goes forward
         * <0 goes backward
         * (dynamic parameter)
         */
        double stepGain;
        /**
         * Vertical rise height of each foot
         * in meters (positive)
         */
        double riseGain;
        /**
         * Angular yaw rotation of each
         * foot for each step in radian.
         * 0 does not turn
         * >0 turns left
         * <0 turns right
         * (dynamic parameter)
         */
        double turnGain;
        /**
         * Lateral length of each foot step
         * in meters.
         * >0 goes left
         * <0 goes right
         * (dynamic parameter)
         */
        double lateralGain;
        /**
         * Vertical foot offset from trunk
         * in meters (positive)
         * 0 is in init position
         * > 0 set the robot lower to the ground
         */
        double trunkZOffset;
        /**
         * Lateral trunk oscillation amplitude
         * in meters (positive)
         */
        double swingGain;
        /**
         * Lateral angular oscillation amplitude
         * of swing trunkRoll in radian
         */
        double swingRollGain;
        /**
         * Phase shift of lateral trunk oscillation
         * between 0 and 1
         */
        double swingPhase;
        /**
         * Foot X-Z spline velocities
         * at ground take off and ground landing.
         * Step stands for X and rise stands for Z
         * velocities.
         * Typical values ranges within 0 and 5.
         * >0 for DownVel is having the foot touching the
         * ground with backward velocity.
         * >0 for UpVel is having the foot going back
         * forward with non perpendicular tangent.
         */
        double stepUpVel;
        double stepDownVel;
        double riseUpVel;
        double riseDownVel;
        /**
         * Time length in phase time
         * where swing lateral oscillation
         * remains on the same side
         * between 0 and 0.5
         */
        double swingPause;
        /**
         * Swing lateral spline velocity (positive).
         * Control the "smoothness" of swing trajectory.
         * Typical values are between 0 and 5.
         */
        double swingVel;
        /**
         * Forward trunk-foot offset
         * with respect to foot in meters
         * >0 moves the trunk forward
         * <0 moves the trunk backward
         */
        double trunkXOffset;
        /**
         * Lateral trunk-foot offset
         * with respect to foot in meters
         * >0 moves the trunk on the left
         * <0 moves the trunk on the right
         */
        double trunkYOffset;
        /**
         * Trunk angular rotation
         * around Y in radian
         * >0 bends the trunk forward
         * <0 bends the trunk backward
         */
        double trunkPitch;
        /**
         * Trunk angular rotation
         * around X in radian
         * >0 bends the trunk on the right
         * <0 bends the trunk on the left
         */
        double trunkRoll;
        /**
         * Add extra offset on X, Y and Z
         * direction on left and right feet
         * in meters
         */
        double extraLeftX;
        double extraLeftY;
        double extraLeftZ;
        double extraRightX;
        double extraRightY;
        double extraRightZ;
        /**
         * Add extra angular offset on
         * Yaw, Pitch and Roll rotation of
         * left and right foot in radians
         */
        double extraLeftYaw;
        double extraLeftPitch;
        double extraLeftRoll;
        double extraRightYaw;
        double extraRightPitch;
        double extraRightRoll;
    };

    class WalkEngine: public singleton<WalkEngine>
    {
    public:
        enum support_foot
        {
            DOUBLE_SUPPORT = 0,
            LEFT_SUPPORT = 1,
            RIGHT_SUPPORT = 2
        };
        WalkEngine();
        ~WalkEngine();
        void start(sensor_ptr s);
        void stop() { is_alive_ = false; }
        void set_enable(const bool &e, const bool &run_action=false)
        {
            e_mutex_.lock();
            enable_ = e;
            if(!enable_)
            {
                para_mutex_.lock();
                params_.stepGain=0.0;
                params_.lateralGain=0.0;
                params_.turnGain=0.0;
                params_.riseGain = 0.0;
                para_mutex_.unlock();
            }
            e_mutex_.unlock();
        }
        void set_params(const Eigen::Vector4f &params);
    private:
        static void boundPhase(double &phase);

        double phase_, dt_;
        support_foot support_foot_;
        void run();
        WalkParameters params_;
        std::thread td_;
        bool is_alive_, enable_;
        std::shared_ptr<motor> motor_;
        mutable std::mutex para_mutex_, e_mutex_;
        Eigen::Vector2d xrange, yrange, drange;
    };

#define WALK WalkEngine::get_singleton()
}

#endif
