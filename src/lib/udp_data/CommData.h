#ifndef UDP_COMM_DATA_H
#define UDP_COMM_DATA_H

#include "model.hpp"

#define COMM_DATA_HEADER "SEU"
struct comm_packet
{
    char header[3];
    unsigned int number=0;
    bool kickoff=false;
    player_info info;
    int state;
};


#endif