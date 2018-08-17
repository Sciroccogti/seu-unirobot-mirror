#ifndef SEU_UNIROBOT_TCP_SERVER_HPP
#define SEU_UNIROBOT_TCP_SERVER_HPP

#include <cstdlib>
#include <deque>
#include <list>
#include <memory>
#include <set>
#include <utility>
#include <boost/asio.hpp>
#include "tcp_packet.hpp"
#include "comm.hpp"
#include "logger.hpp"

namespace comm
{
    using boost::asio::ip::tcp;

    //----------------------------------------------------------------------

    typedef std::deque<tcp_packet> tcp_packet_queue;

    //----------------------------------------------------------------------

    class tcp_connection
    {
    public:
        virtual ~tcp_connection() {}
        virtual void deliver(const tcp_packet&) = 0;
        virtual bool check_type(const tcp_packet::tcp_cmd_type &) = 0;
        virtual void stop()=0;
    };

    typedef std::shared_ptr<tcp_connection> tcp_connection_ptr;

    //----------------------------------------------------------------------

    class tcp_pool
    {
    public:
        void join(tcp_connection_ptr connection)
        {
            connections_.insert(connection);
        }

        void leave(tcp_connection_ptr connection)
        {
            connections_.erase(connection);
            LOG(LOG_INFO, "connection leaved");
        }

        void close()
        {
            for (auto connection : connections_)
            {
                connection->stop();
            }
        }

        void deliver(const tcp_packet &pkt)
        {
            for (auto connection : connections_)
            {
                connection->deliver(pkt);
            }
        }

        void deliver(const tcp_packet::tcp_command cmd)
        {  
            for (tcp_connection_ptr connection : connections_)
            {
                if(connection->check_type(cmd.type))
                {
                    connection->deliver(tcp_packet(cmd));
                }
            }
        }

    private:
        std::set<tcp_connection_ptr> connections_;
    };

    //----------------------------------------------------------------------
    
    class tcp_session: public tcp_connection,
        public std::enable_shared_from_this<tcp_session>
    {
    public:
        tcp_session(tcp::socket socket, tcp_pool &pool, net_comm_callback cb)
            : socket_(std::move(socket)), pool_(pool), cb_(std::move(cb))
        {
            LOG(LOG_INFO, "new connection");
        }

        void start()
        {
            pool_.join(shared_from_this());
            do_read_header();
        }

        void stop()
        {
            socket_.close();
        }

        void deliver(const tcp_packet &pkt)
        {
            bool write_in_progress = !write_pkts_.empty();
            write_pkts_.push_back(pkt);

            if (!write_in_progress)
            {
                do_write();
            }
        }

        bool check_type(const tcp_packet::tcp_cmd_type &t)
        {
            for(auto c:td_map_)
                if(c.first == t && (c.second == tcp_packet::DIR_BOTH || c.second == tcp_packet::DIR_APPLY))
                    return true;
            return false;
        }

    private:
        void do_read_header()
        {
            auto self(shared_from_this());
            boost::asio::async_read(socket_, boost::asio::buffer(read_pkt_.data(), tcp_packet::header_length),
                                    [this, self](boost::system::error_code ec, std::size_t /*length*/)
            {
                if (!ec && read_pkt_.decode_header())
                {
                    do_read_body();
                }
                else
                {
                    pool_.leave(shared_from_this());
                }
            });
        }

        void do_read_body()
        {
            auto self(shared_from_this());
            boost::asio::async_read(socket_, boost::asio::buffer(read_pkt_.body(), read_pkt_.body_length()),
                                    [this, self](boost::system::error_code ec, std::size_t /*length*/)
            {
                if (!ec)
                {
                    tcp_packet::tcp_command cmd = read_pkt_.tcp_cmd();
                    if(cmd.type == tcp_packet::REG_DATA)
                    {
                        tcp_packet::tcp_cmd_type t;
                        tcp_packet::tcp_data_dir d;
                        std::memcpy(&t, cmd.data, tcp_packet::tcp_cmd_type_size);
                        std::memcpy(&d, cmd.data+tcp_packet::tcp_cmd_type_size, tcp_packet::tcp_data_dir_size);
                        td_map_[t] = d;
                    }
                    else
                    {
                        data_.append(read_pkt_.tcp_cmd().data, read_pkt_.tcp_cmd().size);
                        if(!read_pkt_.is_full())
                        {
                            cb_(data_.c_str(), data_.size(), read_pkt_.tcp_cmd().type);
                            data_.clear();
                        }
                    }
                    do_read_header();
                }
                else
                {
                    pool_.leave(shared_from_this());
                }
            });
        }

        void do_write()
        {
            auto self(shared_from_this());
            boost::asio::async_write(socket_, boost::asio::buffer(write_pkts_.front().data(), write_pkts_.front().length()),
                                    [this, self](boost::system::error_code ec, std::size_t /*length*/)
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
                    pool_.leave(shared_from_this());
                }
            });
        }

        tcp::socket socket_;
        tcp_pool &pool_;
        tcp_packet read_pkt_;
        std::map<tcp_packet::tcp_cmd_type, tcp_packet::tcp_data_dir> td_map_;
        tcp_packet_queue write_pkts_;
        net_comm_callback cb_;
        std::string data_;
    };
    
    //----------------------------------------------------------------------

    class tcp_server
    {
    public:
        tcp_server(boost::asio::io_service &io_service, const tcp::endpoint &endpoint, net_comm_callback cb)
            : acceptor_(io_service, endpoint), socket_(io_service), cb_(std::move(cb))
        {
            do_accept();
        }

        void close()
        {
            pool_.close();
        }

        void do_write(const tcp_packet &pkt)
        {
            pool_.deliver(pkt);
        }

    private:
        void do_accept()
        {
            acceptor_.async_accept(socket_, [this](boost::system::error_code ec)
            {
                if (!ec)
                {
                    std::make_shared<tcp_session>(std::move(socket_), pool_, cb_)->start();
                }
                do_accept();
            });
        }

        tcp::acceptor acceptor_;
        tcp::socket socket_;
        tcp_pool pool_;
        net_comm_callback cb_;
    };
}

#endif

