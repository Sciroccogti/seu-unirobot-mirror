#pragma once

#include <boost/asio.hpp>
#include <exception>
#include "task.hpp"
#include "core/worldmodel.hpp"
#include "configuration.hpp"
#include "udp_data/CommData.h"

class say_task: public task
{
public:
    say_task(): task("say")
    {

    }
    
    bool perform()
    {
        static unsigned int pkt_num = 0;
        comm_packet pkt;
        const char* init = COMM_DATA_HEADER;
        for(unsigned int i = 0; i < sizeof(pkt.header); ++i)
            pkt.header[i] = init[i];
        pkt.number = pkt_num++;
        pkt.info = WM->my_info();
        //LOG<<pkt.info.x<<'\t'<<pkt.info.y<<ENDL;

        try
        {
            boost::asio::io_service say_service;
            boost::asio::ip::udp::socket send_socket(say_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)); 
            boost::asio::ip::udp::endpoint send_point(boost::asio::ip::address_v4::broadcast(), CONF->get_config_value<unsigned short>("net.udp.team.port"));
            send_socket.set_option(boost::asio::socket_base::broadcast(true));
            send_socket.async_send_to(boost::asio::buffer((char *)(&pkt), sizeof(comm_packet)), send_point,
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