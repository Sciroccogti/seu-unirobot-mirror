#ifndef SEU_UNIROBOT_ACTUATOR_SAY_PLAN_HPP
#define SEU_UNIROBOT_ACTUATOR_SAY_PLAN_HPP

#include <boost/asio.hpp>
#include "plan.hpp"
#include "robot/humanoid.hpp"
#include "class_exception.hpp"
#include "configuration.hpp"

class say_plan: public plan
{
public:
    say_plan(): plan("say_plan","udp")
    {
    }
    
    int perform(sensor_ptr s=nullptr)
    {
        boost::asio::io_service _ioservice;
        boost::asio::ip::udp::endpoint remote_endpoint(boost::asio::ip::address_v4::from_string(CONF.get_config_value<std::string>("net.udp.addr")),
                                                       CONF.get_config_value<int>("net.udp.team.say_port"));
        try
        {
            boost::asio::ip::udp::socket _socket(_ioservice, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0));
            _socket.set_option(boost::asio::socket_base::broadcast(true));
            //_socket.send_to(boost::asio::buffer((char *)(&_data),
            //                            sizeof(RoboCupGameControlReturnData)), remote_endpoint);
            _socket.close();
            return true;
        }
        catch (std::exception &e)
        {
            std::cerr << e.what() << "\n";
            return false;
        }
    }
private:
    
};

#endif