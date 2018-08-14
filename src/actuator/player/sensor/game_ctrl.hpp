#ifndef SEU_UNIROBOT_GAME_CTRL_HPP
#define SEU_UNIROBOT_GAME_CTRL_HPP

#include <thread>
#include "GameCtrlData/RoboCupGameControlData.h"
#include "sensor.hpp"
#include "comm/udp_server.hpp"

class game_ctrl: public sensor
{
public:
    enum RobocupGameState
    {
        GS_INITIAL, // define STATE_INITIAL 0
        GS_READY, // define STATE_READY 1
        GS_SET, // define STATE_SET 2; !!ATTENTION_PLEASE: DROPBALL will use this pattern
        GS_PLAY, // define STATE_PLAYING 3
        GS_FINISH // define STATE_FINISH 4
    };

    enum RobocupGameSecondaryState
    {
        GS_NORMAL, // define STATE2_NORMAL 0
        GS_PENALTYSHOOT, // define STATE2_PENALTYSHOOT 1
        GS_OVERTIME, // define STATE2_OVERTIME 2
        GS_TIMEOUT, // define STATE2_TIMEOUT 3
        GS_DIRECT_FREEKICK,
        GS_INDIRECT_FREEKICK,
        GS_PENALTYKICK,
    };

    enum RobocupPlayerState
    {
        P_PENALTY_NONE = 0,
        P_PENALTY_PUSHING = 2,
        P_SUBSTITUTE = 15,
        P_PENALTY_BALL_MANIPULATION = 30, // define PENALTY_HL_KID_BALL_MANIPULATION 1
        P_PENALTY_PHYSICAL_CONTACT, // define PENALTY_HL_KID_PHYSICAL_CONTACT 2
        P_PENALTY_ILLEGAL_ATTACK, // define PENALTY_HL_KID_ILLEGAL_ATTACK 3
        P_PENALTY_ILLEGAL_DEFENSE, // define PENALTY_HL_KID_ILLEGAL_DEFENSE 4
        P_PENALTY_PICKUP_OR_INCAPABLE, // defien PENALTY_HL_KID_REQUEST_FOR_PICKUP 5
        P_PENALTY_SERVICE, // define PENALTY_HL_KID_REQUEST_FOR_SERVICE 6
    };
    
    enum FreeKickStage
    {
        FK_ADAJUST=0,
        FK_EXECUTE
    };
    
    game_ctrl(const sub_ptr &s);
    ~game_ctrl();
    bool start();
    void run();
    bool open();
    void close();
    void data_handler(const char* data, const int& size, const int& type);
    
    RoboCupGameControlData data() const
    {
        return *data_;
    }
private:
    enum {gc_data_size = sizeof(RoboCupGameControlData)};
    RoboCupGameControlData *data_;
    std::string str_;
    std::thread td_;
    comm::udp_server server_;
};

#endif