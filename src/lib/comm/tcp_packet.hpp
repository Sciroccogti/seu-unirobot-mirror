#ifndef TCP_PACKET_HPP
#define TCP_PACKET_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "class_exception.hpp"

namespace comm
{
    class tcp_packet
    {
    public:
        enum { header_length = 5 };
        enum { max_body_length = 1024 };

        enum tcp_data_dir
        {
            DIR_BOTH = 0,
            DIR_APPLY = 1,
            DIR_SUPPLY = 2
        };

        enum tcp_cmd_type
        {
            REG_DATA = 0,
            TEST_DATA = 5,
            JOINT_OFFSET = 6,
            WALK_DATA = 7,
            POS_DATA = 8,
            ACT_DATA = 9
        };

        enum {tcp_data_dir_size = sizeof(tcp_data_dir)};
        enum {tcp_cmd_type_size = sizeof(tcp_cmd_type)};

        enum { max_cmd_data_length = max_body_length-tcp_cmd_type_size-sizeof(int)};
        struct tcp_command
        {
            tcp_cmd_type type;
            int size;
            char data[max_cmd_data_length];
        };

        tcp_packet(): body_length_(0), is_full_(false)
        {
        }
        
        tcp_packet(tcp_command cmd)
        {
            body_length_ = cmd.size+sizeof(tcp_cmd_type)+sizeof(int);
            if(body_length_ > max_body_length)
            {
                body_length_ = 0;
                throw class_exception<tcp_packet>("the cmd size is larger than the max length");
            }
            else if(body_length_ == max_body_length) is_full_ = true;
            else is_full_ = false;
            encode_header();
            std::memcpy(body(), (char*)(&cmd), body_length());
        }

        tcp_command tcp_cmd() const
        {
            tcp_command cmd;
            std::memcpy((char*)(&cmd), body(), body_length());
            return cmd;
        }
        
        const bool is_full() const
        {
            return is_full_;
        }

        const char *data() const
        {
            return data_;
        }

        char *data()
        {
            return data_;
        }

        std::size_t length() const
        {
            return header_length + body_length_;
        }

        const char *body() const
        {
            return data_ + header_length;
        }

        char *body()
        {
            return data_ + header_length;
        }

        std::size_t body_length() const
        {
            return body_length_;
        }

        void body_length(std::size_t new_length)
        {
            body_length_ = new_length;

            if (body_length_ > max_body_length)
            {
                body_length_ = max_body_length;
            }
        }

        bool decode_header()
        {
            int body_len;
            std::memcpy(&body_len, data_, sizeof(int));
            //std::cout<<"\ndecode body_len:"<<body_len<<"\n";
            if(body_len>max_body_length) return false;
            body_length_ = body_len;
            is_full_ = static_cast<bool>(*(data_+sizeof(int)));
            //std::cout<<"\ndecode is_full:"<<(int)is_full_<<"\n";
            return true;
        }

        void encode_header()
        {
            int body_len = static_cast<int>(body_length_);
            //std::cout<<"\nencode body_len:"<<body_len<<"\n";
            std::memcpy(data_, (char*)(&body_len), sizeof(int));
            *(data_+sizeof(int)) = static_cast<char>(is_full_);
            //std::cout<<"\nencode is_full:"<<is_full_<<"\n";
        }

    private:
        char data_[header_length + max_body_length];
        std::size_t body_length_;
        bool is_full_;
    };
}

#endif
