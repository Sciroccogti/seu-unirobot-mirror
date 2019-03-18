#pragma once

#include <boost/asio.hpp>
#include <thread>
#include "sensor.hpp"
#include "udp_data/CommData.h"

class hear: public sensor
{
public:
    hear();
    ~hear();

    bool start();
    void stop();

    inline player_info info() const
    {
        return pkt_.info;
    }

private:
    void receive();
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint point_;
    std::thread td_;
    comm_packet pkt_;
};

