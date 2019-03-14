#pragma once

#include <boost/asio.hpp>
#include <thread>
#include "sensor.hpp"
#include "model.hpp"

class hear: public sensor
{
public:
    hear();
    ~hear();

    bool start();
    void stop();
    void send(const player_info &pinfo);

    inline player_info info() const
    {
        return p_info_;
    }

private:
    void receive();
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint point_;
    std::thread td_;
    player_info p_info_;
};

