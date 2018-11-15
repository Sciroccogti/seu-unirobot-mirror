#include "player.hpp"
#include "configuration.hpp"
#include "plan/action_plan.hpp"
#include "plan/lookat_plan.hpp"

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
    if (!regist())
    {
        return false;
    }

    is_alive_ = true;

    if (OPTS->use_robot())
    {
        while (!dynamic_pointer_cast<motor>(sensors_["motor"])->is_connected() && is_alive_)
        {
            std::cout << "\033[33mwaitint for motor connection, please turn on the power.\033[0m" << std::endl;
            sleep(1);
        }

        if (!is_alive_)
        {
            return true;
        }
    }

    MADT->start();

    actuators_["body"] = std::make_shared<actuator>("body");
    actuators_["head"] = std::make_shared<actuator>("head");

    for (auto &a : actuators_)
    {
        a.second->start();
    }

    action_plan p("ready");
    p.perform();

    fsm_ = make_shared<fsm>();
    start_timer();
    WE->start();
    return true;
}

void player::stop()
{
    WE->stop();
    MADT->stop();

    for (auto &a : actuators_)
    {
        a.second->stop();
    }

    if (is_alive_)
    {
        delete_timer();
    }

    is_alive_ = false;
    sleep(1);
    unregist();
}

void player::run()
{
    if (is_alive_)
    {
        period_count_++;


        if (WM->lost())
        {
            std::cout << "\033[31mhardware has no response!\033[0m\n";
            raise(SIGINT);
        }

        if (OPTS->use_robot())
        {
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

        add_plans(think());
    }
}

list< plan_ptr > player::think()
{
    list<plan_ptr> plist;
    list<plan_ptr> tlist;

    if (period_count_ * period_ms_ % 1000 == 0)
    {
        if (WM->low_power())
        {
            std::cout << "\033[31m******** low power! ********\033[0m\n";
        }
    }

    if (OPTS->use_remote())
    {
        tlist = play_with_remote();
    }
    else
    {
        if (OPTS->use_gc())
        {
            tlist = play_with_gamectrl();
        }
        else
        {
            tlist = play_without_gamectrl();
        }
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
            std::cout << "cannot find actuator: " + p->actuator_name() << "\n";
        }
    }
}

bool player::regist()
{
    sensors_.clear();

    if (OPTS->use_debug())
    {
        sensors_["server"] = std::make_shared<tcp_server>();
        sensors_["server"]->attach(WM);
        sensors_["server"]->start();
    }

    if (OPTS->use_camera())
    {
        sensors_["camera"] = std::make_shared<camera>();
        sensors_["camera"]->attach(VISION);
        sensors_["camera"]->start();

        if (!VISION->start(get_sensor("server")))
        {
            return false;
        }
    }

    sensors_["motor"] = std::make_shared<motor>(get_sensor("server"));
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

    if (OPTS->use_gc())
    {
        try
        {
            sensors_["gc"] = std::make_shared<gamectrl>();
            sensors_["gc"]->attach(WM);
            sensors_["gc"]->start();
        }
        catch (std::exception &e)
        {
            std::cout << "\033[33mgame controller: " << e.what() << "\033[0m\n";
        }
    }

    if (OPTS->use_comm())
    {
        try
        {
            sensors_["hear"] = std::make_shared<hear>();
            sensors_["hear"]->attach(WM);
            sensors_["hear"]->start();
        }
        catch (std::exception &e)
        {
            std::cout << "say_hear: " << e.what() << "\n";
        }
    }

    return true;
}

void player::unregist()
{
    if (sensors_.find("gc") != sensors_.end())
    {
        sensors_["gc"]->detach(WM);
        sensors_["gc"]->stop();
    }

    if (sensors_.find("hear") != sensors_.end())
    {
        sensors_["hear"]->detach(WM);
        sensors_["hear"]->stop();
    }

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

    if (sensors_.find("server") != sensors_.end())
    {
        sensors_["server"]->detach(WM);
        sensors_["server"]->stop();
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