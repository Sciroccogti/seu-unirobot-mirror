#include "player.hpp"
#include "configuration.hpp"
#include "plan/action_plan.hpp"
#include "plan/lookat_plan.hpp"
#include "plan/say_plan.hpp"

using namespace std;
using namespace FSM;
using namespace walk;

player::player(): timer(CONF->get_config_value<int>("think_period"))
{
    is_alive_ = false;
    period_count_ = 0;
}

bool player::init()
{
    if(!regist()) return false;
    if(OPTS->use_robot() != options::ROBOT_NONE)
    {
        actuators_["body"] = std::make_shared<actuator>("body", get_sensor("motor"));
        actuators_["head"] = std::make_shared<actuator>("head", get_sensor("motor"));
    }
    for(auto a:actuators_)
        a.second->start();

    is_alive_ = true;
    fsm_ = make_shared<fsm>();
    start_timer();
    //WALK->start(suber_->get_sensor("motor"));
    return true;
}

void player::stop()
{
    for(auto &a:actuators_)
        a.second->stop();
    if(is_alive_) delete_timer();
    is_alive_ = false;
    sleep(1);
    unregist();
}

void player::run()
{
    if(is_alive_)
    {
        period_count_++;
        add_plan(think());
    }
}

list< plan_ptr > player::think()
{
    list<plan_ptr> plist;
    list<plan_ptr> tlist;
    if(OPTS->use_remote())
    {
        tlist = play_with_remote();
    }
    else
    {
        if(OPTS->use_gc())
            tlist = play_with_gamectrl();
        else
            tlist = play_without_gamectrl();
    }
    plist.insert(plist.end(), tlist.begin(), tlist.end());
    return plist;
}

void player::add_plan(std::list<plan_ptr> plist)
{
    for(auto &p:plist)
    {
        if(actuators_.find(p->actuator_name()) != actuators_.end())
        {
            actuator_ptr actu = actuators_[p->actuator_name()];
            actu->add_plan(p);
        }
        else
        {
            std::cout<<"cannot find actuator: "+p->actuator_name()<<"\n";
        }
    }
}

bool player::regist()
{
    wm_ = std::make_shared<worldmodel>();
    we_ = std::make_shared<walk::WalkEngine>();

    sensors_.clear();
    if(OPTS->use_debug())
    {
        sensors_["server"] = std::make_shared<tcp_server>();
        sensors_["server"]->attach(wm_);
        sensors_["server"]->start();
    }
    if(OPTS->use_camera())
    {
        vision_ = std::make_shared<vision>(get_sensor("server"));
        sensors_["camera"] = std::make_shared<camera>();
        sensors_["camera"]->attach(vision_);
        sensors_["camera"]->start();
        if(!vision_->start()) return false;
    }
    if(OPTS->use_robot() == options::ROBOT_REAL)
    {
        try
        {
            sensors_["imu"] = std::make_shared<imu>();
            sensors_["imu"]->attach(wm_);
            sensors_["imu"]->attach(we_);
            sensors_["imu"]->start();
        }
        catch(std::exception &e)
        {
            std::cout<<e.what()<<"\n";
        }
        sensors_["motor"] = std::make_shared<rmotor>();
        sensors_["motor"]->attach(wm_);
        sensors_["motor"]->attach(we_);
        if(!sensors_["motor"]->start()) return false;
    }
    else if(OPTS->use_robot() == options::ROBOT_VIRTUAL)
    {
        if(OPTS->use_debug())
        {
            sensors_["motor"] = std::make_shared<vmotor>(sensors_["server"]);
            if(!sensors_["motor"]->start()) return false;
        }
        else
        {
            std::cout<<"If you want to use virtual robot, you must run with -d1 -r2 \n";
            return false;
        }
    }
    if(OPTS->use_gc())
    {
        try
        {
            sensors_["gc"] = std::make_shared<gamectrl>();
            sensors_["gc"]->attach(wm_);
            sensors_["gc"]->start();
        }
        catch(std::exception &e)
        {
            std::cout<<e.what()<<"\n";
        }
    }
    if(OPTS->use_comm())
    {
        try
        {
            sensors_["hear"] = std::make_shared<hear>();
            sensors_["hear"]->attach(wm_);
            sensors_["hear"]->start();
        }
        catch(std::exception &e)
        {
            std::cout<<e.what()<<"\n";
        }
    }
    return true;
}

void player::unregist()
{
    if(sensors_.find("gc") != sensors_.end())
    {
        sensors_["gc"]->detach(wm_);
        sensors_["gc"]->stop();
    }
    if(sensors_.find("hear") != sensors_.end())
    {
        sensors_["hear"]->detach(wm_);
        sensors_["hear"]->stop();
    }
    if(sensors_.find("imu") != sensors_.end())
    {
        sensors_["imu"]->detach(wm_);
        sensors_["imu"]->detach(we_);
        sensors_["imu"]->stop();
    }
    if(sensors_.find("motor") != sensors_.end())
    {
        if(OPTS->use_robot() == options::ROBOT_REAL)
        {
            sensors_["motor"]->detach(wm_);
            sensors_["motor"]->detach(we_);
        }
        sensors_["motor"]->stop();
    }
    if(sensors_.find("camera") != sensors_.end())
    {
        sensors_["camera"]->detach(vision_);
        sensors_["camera"]->stop();
        vision_->stop();
    }
    if(sensors_.find("server") != sensors_.end())
    {
        sensors_["server"]->detach(wm_);
        sensors_["server"]->stop();
    }
}

sensor_ptr player::get_sensor(const std::string &name)
{
    auto iter = sensors_.find(name);
    if(iter != sensors_.end())
        return iter->second;
    return nullptr;
}