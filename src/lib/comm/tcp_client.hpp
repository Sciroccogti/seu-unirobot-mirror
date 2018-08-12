#ifndef TCP_CLIENT_HPP
#define TCP_CLIENT_HPP

#include <cstdlib>
#include <deque>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "tcp_packet.hpp"
#include "comm.hpp"

namespace comm
{
    using boost::asio::ip::tcp;
    typedef std::deque<tcp_packet> tcp_packet_queue;

    class tcp_client
    {
    public:
        tcp_client(boost::asio::io_service &io_service, const std::string &host, const int &port, tcp_comm_callback cb=nullptr)
            : io_service_(io_service), socket_(io_service), timer_(io_service), host_(host), port_(std::to_string(port)), cb_(std::move(cb))
        {
            connected_ = false;
            timer_.expires_at(boost::posix_time::pos_infin);
            do_connect(tcp::resolver(io_service_).resolve({host_.c_str(), port_.c_str()}));
        }

        void write(const tcp_packet::tcp_command &cmd)
        {
            write(tcp_packet(cmd));
        }

        void write(const tcp_packet::tcp_cmd_type &type, int size, const char *data)
        {
            tcp_packet::tcp_command cmd;
            cmd.type = type;
            int i=0;
            while(size >= tcp_packet::max_cmd_data_length)
            {
                cmd.size = tcp_packet::max_cmd_data_length;
                std::memcpy(cmd.data, data+i*tcp_packet::max_cmd_data_length, tcp_packet::max_cmd_data_length);
                i++;
                size -= tcp_packet::max_cmd_data_length;
                write(tcp_packet(cmd));
                usleep(10);
            }
            cmd.size = size;
            std::memcpy(cmd.data, data+i*tcp_packet::max_cmd_data_length, size);
            write(tcp_packet(cmd));
        }

        void close()
        {
            io_service_.post([this]()
            {
                socket_.close();
            });
        }

        bool is_connected() const { return connected_; }

        void time_out()
        {
            if(!connected_)
            {
                socket_.cancel();
                do_connect(tcp::resolver(io_service_).resolve({host_.c_str(), port_.c_str()}));
            }
        }

    private:
        void do_connect(tcp::resolver::iterator endpoint_iterator)
        {
            timer_.expires_from_now(boost::posix_time::seconds(1));
            boost::asio::async_connect(socket_, endpoint_iterator,
                                    [this](boost::system::error_code ec, tcp::resolver::iterator)
            {
                if (!ec)
                {
                    connected_ = true;
                    do_read_header();
                }
            });
            timer_.async_wait(std::bind(&tcp_client::time_out, this));
        }

        void do_read_header()
        {
            boost::asio::async_read(socket_, boost::asio::buffer(read_pkt_.data(), tcp_packet::header_length),
                                    [this](boost::system::error_code ec, std::size_t /*length*/)
            {
                if (!ec && read_pkt_.decode_header())
                {
                    do_read_body();
                }
                else
                {
                    socket_.close();
                    connected_ = false;
                    do_connect(tcp::resolver(io_service_).resolve({host_.c_str(), port_.c_str()}));
                }
            });
        }

        void do_read_body()
        {
            boost::asio::async_read(socket_, boost::asio::buffer(read_pkt_.body(), read_pkt_.body_length()),
                                    [this](boost::system::error_code ec, std::size_t /*length*/)
            {
                if (!ec)
                {
                    data_.append(read_pkt_.tcp_cmd().data, read_pkt_.tcp_cmd().size);
                    if(!read_pkt_.is_full())
                    {
                        if(cb_!= nullptr) cb_(data_.c_str(), data_.size(), read_pkt_.tcp_cmd().type);
                        data_.clear();
                    }
                    do_read_header();
                }
                else
                {
                    socket_.close();
                    connected_ = false;
                    do_connect(tcp::resolver(io_service_).resolve({host_.c_str(), port_.c_str()}));
                }
            });
        }

        void do_write()
        {
            boost::asio::async_write(socket_, boost::asio::buffer(write_pkts_.front().data(), write_pkts_.front().length()),
                                    [this](boost::system::error_code ec, std::size_t /*length*/)
            {
                if (!ec)
                {
                    write_pkts_.pop_front();

                    if (!write_pkts_.empty())
                    {
                        do_write();
                    }
                }
                else
                {
                    socket_.close();
                    connected_ = false;
                    do_connect(tcp::resolver(io_service_).resolve({host_.c_str(), port_.c_str()}));
                }
            });
        }
        
        void write(const tcp_packet &pkt)
        {
            io_service_.post([this, pkt]()
            {
                bool write_in_progress = !write_pkts_.empty();
                write_pkts_.push_back(pkt);

                if (!write_in_progress)
                {
                    do_write();
                }
            });
        }

    private:
        boost::asio::io_service &io_service_;
        tcp::socket socket_;
        tcp_packet read_pkt_;
        tcp_packet_queue write_pkts_;
        std::string data_;
        bool connected_;
        boost::asio::deadline_timer timer_;
        std::string host_, port_;
        tcp_comm_callback cb_;
    };
}
#endif
