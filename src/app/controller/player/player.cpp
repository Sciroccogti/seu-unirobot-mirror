#include "player.hpp"
#include "configuration.hpp"
#include "task/action_task.hpp"
#include "task/look_task.hpp"
#include "server/server.hpp"
#include "core/adapter.hpp"
#include "motion/walk/WalkEngine.hpp"
#include "motion/scan/ScanEngine.hpp"
#include "motion/action/ActionEngine.hpp"
using namespace std;
using namespace motion;

player::player(): timer(CONF->get_config_value<int>("think_period"))
{
    is_alive_ = false;
    period_count_ = 0;
}

bool player::init()
{
    SERVER->start();

    if (!regist())
    {
        return false;
    }

    is_alive_ = true;

    if (OPTS->use_robot())
    {
        while (!dynamic_pointer_cast<motor>(sensors_["motor"])->is_connected() && is_alive_)
        {
            LOG << "waiting for motor connection, please turn on the power." << ENDL;
            sleep(1);
        }

        if (!is_alive_)
        {
            return true;
        }
    }

    MADT->start();
    WE->start();
    SE->start();
    AE->start();
    action_task p("ready");
    p.perform();
    start_timer();
    return true;
}

void player::stop()
{
    WE->stop();
    SE->stop();
    AE->stop();
    MADT->stop();
/*
    for (auto &a : actuators_)
    {
        a.second->stop();
    }
*/
    if (is_alive_)
    {
        delete_timer();
    }

    is_alive_ = false;
    sleep(1);
    unregist();
    SERVER->stop();
}

void player::run()
{
    if (is_alive_)
    {
        period_count_++;

        tcp_command cmd;
        int fall = WM->fall_data();
        cmd.type = WM_DATA;
        cmd.size = 5 * float_size + bool_size;
        cmd.data.append((char *) & (WM->ballx_), float_size);
        cmd.data.append((char *) & (WM->bally_), float_size);
        cmd.data.append((char *) & (WM->bodyx_), float_size);
        cmd.data.append((char *) & (WM->bodyy_), float_size);
        cmd.data.append((char *) & (WM->bodydir_), float_size);
        cmd.data.append((char *) & fall, int_size);
        SERVER->write(cmd);

        if (OPTS->use_robot())
        {
            if (WM->lost())
            {
                LOG << "hardware has no response!" << ENDL;
                raise(SIGINT);
            }
            if (WM->switch_data().sw1 && !WM->switch_data().sw2)
            {
                dynamic_pointer_cast<imu>(get_sensor("imu"))->set_led_state(LED_WARNING);
            }
            else if (WM->switch_data().sw2 && !WM->switch_data().sw1)
            {
                dynamic_pointer_cast<imu>(get_sensor("imu"))->set_led_state(LED_ERROR);
            }
            else
            {
                if (WM->switch_data().sw2 && WM->switch_data().sw1)
                {
                    dynamic_pointer_cast<imu>(get_sensor("imu"))->set_zero();
                }

                dynamic_pointer_cast<imu>(get_sensor("imu"))->set_led_state(LED_NORMAL);
            }
        }
        list<task_ptr> tasks = SERVER->tasks();
        for(auto &tsk: tasks)
        {
            tsk->perform();
        }
        //add_plans(think());
    }
}

/*
list< plan_ptr > player::think()
{
    list<plan_ptr> plist;
    list<plan_ptr> tlist;

    if (period_count_ * period_ms_ % 1000 == 0)
    {
        if (WM->low_power())
        {
            LOG << "******** low power! ********" << ENDL;
        }
    }

    if (OPTS->use_remote())
    {
        tlist = play_with_remote();
    }
    else
    {
        tlist = SERVER->plans();
    }

    plist.insert(plist.end(), tlist.begin(), tlist.end());
    return plist;
}

void player::add_plans(std::list<plan_ptr> plist)
{
    for (auto &p : plist)
    {
        if (actuators_.find(p->actuator_name()) != actuators_.end())
        {
            actuator_ptr actu = actuators_[p->actuator_name()];
            actu->add_plan(p);
        }
        else
        {
            LOG << "cannot find actuator: " + p->actuator_name() << ENDL;
        }
    }
}
*/
bool player::regist()
{
    sensors_.clear();

    if (OPTS->use_camera())
    {
        sensors_["camera"] = std::make_shared<camera>();
        sensors_["camera"]->attach(VISION);
        sensors_["camera"]->start();

        if (!VISION->start())
        {
            return false;
        }
    }

    sensors_["motor"] = std::make_shared<motor>();
    sensors_["motor"]->attach(WM);
    sensors_["motor"]->attach(WE);

    if (!sensors_["motor"]->start())
    {
        return false;
    }

    if (OPTS->use_robot())
    {
        sensors_["imu"] = std::make_shared<imu>();
        sensors_["imu"]->attach(WM);
        sensors_["imu"]->attach(WE);
        sensors_["imu"]->start();
    }

    return true;
}

void player::unregist()
{
    if (sensors_.find("imu") != sensors_.end())
    {
        sensors_["imu"]->detach(WM);
        sensors_["imu"]->detach(WE);
        sensors_["imu"]->stop();
    }

    if (sensors_.find("motor") != sensors_.end())
    {
        sensors_["motor"]->detach(WM);
        sensors_["motor"]->detach(WE);
        sensors_["motor"]->stop();
    }

    if (sensors_.find("camera") != sensors_.end())
    {
        sensors_["camera"]->detach(VISION);
        sensors_["camera"]->stop();
        VISION->stop();
    }
}

sensor_ptr player::get_sensor(const std::string &name)
{
    auto iter = sensors_.find(name);

    if (iter != sensors_.end())
    {
        return iter->second;
    }

    return nullptr;
}