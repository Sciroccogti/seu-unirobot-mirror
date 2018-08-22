#ifndef SEU_UNIROBOT_TCP_HPP
#define SEU_UNIROBOT_TCP_HPP

#include <string>
#include <vector>

#define MAX_CMD_LEN 512

enum tcp_data_dir
{
    DIR_BOTH = 0,
    DIR_APPLY = 1,
    DIR_SUPPLY = 2
};

enum tcp_cmd_type
{
    NONE_DATA = 0,
    BEAT_DATA = 1,
    REG_DATA = 2,
    TEST_DATA = 5,
    JOINT_DATA = 6,
    REMOTE_DATA = 7,
    IMAGE_DATA = 8
};

struct tcp_command
{
    tcp_cmd_type type;
    unsigned char end;
    unsigned int size;
    std::string data;
};

enum remote_data_type
{
    NON_DATA = 0,
    WALK_DATA = 1,
    ACT_DATA = 2,
    JOINT_OFFSET = 3
};

struct remote_data
{
    remote_data_type type;
    unsigned int size;
    std::string data;
};

enum {tcp_dir_size = sizeof(tcp_data_dir)};
enum {tcp_type_size = sizeof(tcp_cmd_type)};
enum {tcp_size_size = sizeof(unsigned int)};
enum {tcp_full_size = sizeof(unsigned char)};
enum {rmt_type_size = sizeof(remote_data_type)};
enum {rmt_size_size = sizeof(unsigned int)};
enum {float_size = sizeof(float)};
enum {int_size = sizeof(int)};
        
typedef std::function<void (const tcp_command)> tcp_callback;

#endif