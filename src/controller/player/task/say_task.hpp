#pragma once

#include <boost/asio.hpp>
#include <exception>
#include "task.hpp"
#include "core/worldmodel.hpp"
#include "configuration.hpp"

class say_task: public task
{
public:
    say_task(): task("say")
    {

    }
    
    bool perform()
    {
        player_info info = WM->my_info();

        boost::asio::io_service say_service;
        boost::asio::ip::udp::socket send_socket(say_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)); 
        boost::asio::ip::udp::endpoint send_point(boost::asio::ip::address_v4::broadcast(), CONF->get_config_value<unsigned short>("net.udp.teammate.port"));
        send_socket.set_option(boost::asio::socket_base::broadcast(true));

        try
        {
            send_socket.async_send_to(boost::asio::buffer((char *)(&info), sizeof(player_info)), send_point,
            [this](boost::system::error_code ec, std::size_t bytes_sent) {});
            return true;
        }
        catch (std::exception &e)
        {
            LOG << e.what() << ENDL;
            return false;
        }
    }
};