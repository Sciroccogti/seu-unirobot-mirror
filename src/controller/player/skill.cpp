#include "skill.hpp"
#include "task/action_task.hpp"
#include "task/look_task.hpp"
#include "task/walk_task.hpp"

using namespace std;
using namespace Eigen;
using namespace robot_math;
using namespace robot;

const double skill_goto_max_omnidirectional_distance = 0.5;
const double skill_goto_max_speed = 0.03;
const double skill_goto_stop_distance = 0.3;
const double skill_goto_stop_direction = 10.0;


task_ptr skill_goto_forward(const Vector2d &p, double dir);
task_ptr skill_goto_omnidirectional(const Vector2d &p, double dir);

task_ptr skill_track_ball()
{
    ball_block ball = WM->ball();
    std::vector<double> head_degs = ROBOT->get_head_degs();
    float curYawDeg = head_degs[0];

    if (fabs(ball.alpha) > 0.2f)
    {
        curYawDeg += -(sign(ball.alpha) * 10.0f);
    }
        
    if (ball.self.x() < 0.8f)
    {
        return make_shared<look_task>(curYawDeg, 60.0, false);
    }
    else
    {
        return make_shared<look_task>(curYawDeg, 30.0, true);
    }
}

task_ptr skill_goto(const Vector2d &p, double dir, bool omnidirectional)
{
    self_block me = WM->self();
    if(!omnidirectional&&(me.global-p).norm()>skill_goto_max_omnidirectional_distance)
    {
        return skill_goto_forward(p, dir);
    }
    else
    {
        return skill_goto_omnidirectional(p, dir);
    }
    
}

task_ptr skill_goto_forward(const Vector2d &p, double dir)
{
    self_block me = WM->self();
    double walkDir = azimuth((p - me.global));
    float drift_angle = normalize_deg(walkDir - me.dir);

    if (fabs(drift_angle) > 20.0f)
    {
        return make_shared<walk_task>(0,0,10.0,true);
    }
    else
    {
        return make_shared<walk_task>(skill_goto_max_speed,0,0,true);
    }
}

task_ptr skill_goto_omnidirectional(const Vector2d &p, double dir)
{
    self_block me=WM->self();
    double distance = (me.global-p).norm();
    double drift_angle = normalize_deg(dir-me.dir);
    if (distance < skill_goto_stop_distance && fabs(drift_angle) < skill_goto_stop_direction)
        return make_shared<task>();

    Vector2d relativeTarget = p-me.global;
    return make_shared<walk_task>(relativeTarget.x(), relativeTarget.y(), drift_angle, true);
}

task_ptr skill_kick()
{
    
}

task_ptr skill_goto_behind_the_ball(const Vector2f &relativeOffset, const Vector2f &aimAt)
{
    return make_shared<task>();
    /*
    ball_block ball = WM->ball();
    float aimAngle = azimuth(aimAt - ball.global);
    TransMatrixf trans;
    trans.identity();
    trans.transfer(Vector3f(ball.GlobalPos2D.x(), ball.GlobalPos2D.y(), 0));
    trans.rotateLocalZ(aimAngle);
    Vector3f offensivePosition3D = trans.transform(Vector3f(relativeOffset.x(), relativeOffset.y(), 0));

    Vector2f offensivePosition(offensivePosition3D.x(), offensivePosition3D.y());
    //avoid ball
    const float ballDistance = ball.RelPos2D.length();
    const float avoidAngle = atan2Deg(skill_avoid_ball_distance, ballDistance);
    const float leftAvoidAngle = ball.RelPos2D.angle() + avoidAngle;
    const float rightAvoidAngle = ball.RelPos2D.angle() - avoidAngle;
    self_block me = WM->self();
    const Vector2f offensivePositionRel = offensivePosition-me.global;

    p->_strategyInfomation.test = offensivePosition;
    if (offensivePositionRel.angle() > rightAvoidAngle 
        && offensivePositionRel.angle() < leftAvoidAngle && offensivePositionRel.length() > ballDistance)
    {
        const AngDeg ballAngle = ball.RelPos2D.angle();
        const AngDeg avoidAngle = (offensivePositionRel.angle() > ball.RelPos2D.angle()) ? leftAvoidAngle
                                    : rightAvoidAngle;
        Vector2f avoidPositionRel(ballDistance * cosDeg(avoidAngle), ballDistance * sinDeg(avoidAngle));
        Vector2f avoidPosition = WM.transRelPos2DToGlobalPos2D(avoidPositionRel);
        PLAN plan =skill_goto(p, avoidPosition, (ball.GlobalPos2D - me.globalPos2D).angle());
        if(plan.get()) return plan;
        return skill_goto(p, avoidPosition, aimAngle);
    }
    PLAN plan =skill_goto(p, offensivePosition, (ball.GlobalPos2D - me.globalPos2D).angle());
    if(plan.get()) return plan;
    return skill_goto(p, offensivePosition, aimAngle);
    */
}