#ifndef SEU_UNIROBOT_DYNAMIXEL_H
#define SEU_UNIROBOT_DYNAMIXEL_H

#include <boost/asio.hpp>
#include "timer.hpp"
#include "singleton.hpp"
#include "communication/serial_port_handler.hpp"

#define TXPACKET_MAX_LEN    (4*1024)
#define RXPACKET_MAX_LEN    (4*1024)

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

class dynamixel
{
public:
    dynamixel(const std::string &dev_name, const int &baudrate);
    void sync_write(uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length);
    bool sync_read(uint16_t start_address, uint16_t data_length, std::map<uint8_t, uint8_t*> data);
    void write(uint8_t id, uint16_t address, uint16_t length, uint8_t *data);
    bool read(uint8_t id, uint16_t address, uint16_t length, uint8_t *data);
private:
    unsigned short update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
    void add_stuffing(uint8_t *packet);
    void remove_stuffing(uint8_t *packet);
    void tx_packet(uint8_t *txpacket);
    bool rx_packet(uint8_t *rxpacket, const int &length);

    serial_port_handler serial_port_;
};
#endif