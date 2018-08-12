#include "dynamixel.hpp"
#include "configuration.hpp"

#define BROADCAST_ID        0xFE    // 254
#define MAX_ID              0xFC    // 252

/* Instruction for DXL Protocol */
#define INST_PING               1
#define INST_READ               2
#define INST_WRITE              3
#define INST_REG_WRITE          4
#define INST_ACTION             5
#define INST_FACTORY_RESET      6
#define INST_SYNC_WRITE         131     // 0x83
#define INST_BULK_READ          146     // 0x92
#define INST_REBOOT             8
#define INST_STATUS             85      // 0x55
#define INST_SYNC_READ          130     // 0x82
#define INST_BULK_WRITE         147     // 0x93

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

///////////////// Protocol 2.0 Error bit /////////////////
#define ERRNUM_RESULT_FAIL      1       // Failed to process the instruction packet.
#define ERRNUM_INSTRUCTION      2       // Instruction error
#define ERRNUM_CRC              3       // CRC check error
#define ERRNUM_DATA_RANGE       4       // Data range error
#define ERRNUM_DATA_LENGTH      5       // Data length error
#define ERRNUM_DATA_LIMIT       6       // Data limit error
#define ERRNUM_ACCESS           7       // Access error

#define ERRBIT_ALERT            128     //When the device has a problem, this bit is set to 1. Check "Device Status Check" valu

using namespace std;

dynamixel::dynamixel(const std::string &dev_name, const int &baudrate): serial_port_(dev_name, baudrate)
{
    serial_port_.start();
}

unsigned short dynamixel::update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i;
    uint16_t crc_table[256] = {0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };

    for (uint16_t j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

void dynamixel::add_stuffing(uint8_t *packet)
{
    int i = 0, index = 0;
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;
    uint8_t temp[TXPACKET_MAX_LEN] = {0};

    for (uint16_t s = PKT_HEADER0; s <= PKT_LENGTH_H; s++)
    temp[s] = packet[s]; // FF FF FD XX ID LEN_L LEN_H
    //memcpy(temp, packet, PKT_LENGTH_H+1);
    index = PKT_INSTRUCTION;
    for (i = 0; i < packet_length_in - 2; i++)  // except CRC
    {
        temp[index++] = packet[i+PKT_INSTRUCTION];
        if (packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
        {   // FF FF FD
            temp[index++] = 0xFD;
            packet_length_out++;
        }
    }
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
    temp[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];

    if (packet_length_in != packet_length_out)
    packet = (uint8_t *)realloc(packet, index * sizeof(uint8_t));

    for (uint16_t s = 0; s < index; s++)
    packet[s] = temp[s];
    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

void dynamixel::remove_stuffing(uint8_t *packet)
{
  int i = 0, index = 0;
  int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
  int packet_length_out = packet_length_in;

  index = PKT_INSTRUCTION;
  for (i = 0; i < packet_length_in - 2; i++)  // except CRC
  {
    if (packet[i+PKT_INSTRUCTION] == 0xFD && packet[i+PKT_INSTRUCTION+1] == 0xFD && packet[i+PKT_INSTRUCTION-1] == 0xFF && packet[i+PKT_INSTRUCTION-2] == 0xFF)
    {   // FF FF FD FD
      packet_length_out--;
      i++;
    }
    packet[index++] = packet[i+PKT_INSTRUCTION];
  }
  packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-2];
  packet[index++] = packet[PKT_INSTRUCTION+packet_length_in-1];

  packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
  packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

void dynamixel::tx_packet(uint8_t *txpacket)
{
    uint16_t total_packet_length   = 0;
    uint16_t written_packet_length = 0;
    // byte stuffing for header
    add_stuffing(txpacket);
    // check max packet length
    total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;
    // 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H
    if (total_packet_length > TXPACKET_MAX_LEN) return;
    // make packet header
    txpacket[PKT_HEADER0]   = 0xFF;
    txpacket[PKT_HEADER1]   = 0xFF;
    txpacket[PKT_HEADER2]   = 0xFD;
    txpacket[PKT_RESERVED]  = 0x00;
    // add CRC16
    uint16_t crc = update_crc(0, txpacket, total_packet_length - 2);    // 2: CRC16
    txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
    txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);
    // tx packet
    serial_port_.write(txpacket, total_packet_length);
}

bool dynamixel::rx_packet(uint8_t *rxpacket, const int &length)
{
    if(rxpacket == nullptr) return false;
    bool res = serial_port_.read(rxpacket, length, 10);
    if(!res) return false;

    uint16_t crc = DXL_MAKEWORD(rxpacket[length-2], rxpacket[length-1]);
    if (update_crc(0, rxpacket, length - 2) == crc)
    {
        remove_stuffing(rxpacket);
        return true;
    }
    else return false;
}

void dynamixel::sync_write(uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length)
{
    uint8_t *txpacket           = (uint8_t *)malloc(param_length + 14);
    // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_INSTRUCTION]   = INST_SYNC_WRITE;
    txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(start_address);
    txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(start_address);
    txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(data_length);
    txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(data_length);

    for (uint16_t i = 0; i < param_length; i++)
        txpacket[PKT_PARAMETER0+4+i] = param[i];
    tx_packet(txpacket);
    free(txpacket);
}

bool dynamixel::sync_read(uint16_t start_address, uint16_t data_length, std::map<uint8_t, uint8_t*> data)
{
    uint16_t param_length = data.size();
    uint8_t *txpacket           = (uint8_t *)malloc(param_length + 14);
    // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_ID]            = BROADCAST_ID;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
    txpacket[PKT_INSTRUCTION]   = INST_SYNC_READ;
    txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(start_address);
    txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(start_address);
    txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(data_length);
    txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(data_length);

    auto iter = data.begin();
    uint16_t i=0;
    while(iter != data.end())
    {
        if(iter->second == nullptr) return false;
        txpacket[PKT_PARAMETER0+4+i] = iter->first;
        iter++;
        i++;
    }
    tx_packet(txpacket);
    free(txpacket);
    unsigned int rx_length = data_length + 11;
    uint8_t *rxpacket = (uint8_t *)malloc(rx_length);
    for(i=0;i<data.size();i++)
    {
        memset(rxpacket, 0, rx_length);
        if(rx_packet(rxpacket, rx_length))
            memcpy(data[rxpacket[PKT_ID]], rxpacket+PKT_PARAMETER0, data_length);
        else return false;
    }
    return true;
}

bool dynamixel::read(uint8_t id, uint16_t address, uint16_t length, uint8_t *data)
{
    if(data == nullptr) return false;
    uint8_t txpacket[14]        = {0};
    if (id >= BROADCAST_ID) return false;
    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = 7;
    txpacket[PKT_LENGTH_H]      = 0;
    txpacket[PKT_INSTRUCTION]   = INST_READ;
    txpacket[PKT_PARAMETER0+0]  = (uint8_t)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER0+1]  = (uint8_t)DXL_HIBYTE(address);
    txpacket[PKT_PARAMETER0+2]  = (uint8_t)DXL_LOBYTE(length);
    txpacket[PKT_PARAMETER0+3]  = (uint8_t)DXL_HIBYTE(length);
    tx_packet(txpacket);
    unsigned int rx_length = length + 11;
    uint8_t *rxpacket = (uint8_t *)malloc(rx_length);
    bool res = rx_packet(rxpacket, rx_length);
    if(res)
    {
        for (uint16_t i = 0; i < length; i++)
            data[i] = rxpacket[PKT_PARAMETER0 + 1 + i];
    }
    free(rxpacket);
    return res;
}

void dynamixel::write(uint8_t id, uint16_t address, uint16_t length, uint8_t *data)
{
    uint8_t *txpacket           = (uint8_t *)malloc(length+12);
    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5);
    txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5);
    txpacket[PKT_INSTRUCTION]   = INST_WRITE;
    txpacket[PKT_PARAMETER0+0]  = (uint8_t)DXL_LOBYTE(address);
    txpacket[PKT_PARAMETER0+1]  = (uint8_t)DXL_HIBYTE(address);

    for (uint16_t i = 0; i < length; i++)
        txpacket[PKT_PARAMETER0+2+i] = data[i];
    tx_packet(txpacket);
    free(txpacket);
}