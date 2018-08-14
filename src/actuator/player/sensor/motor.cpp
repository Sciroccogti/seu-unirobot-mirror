#include "motor.hpp"
#include "configuration.hpp"
#include "options/options.hpp"
#include "tcp_server/tcp_server_handler.hpp"

#define ADDR_ID     7
#define ADDR_TORQ   64
#define ADDR_LED    65
#define ADDR_GPOS   116
#define ADDR_PPOS   132
#define ADDR_VOLT   144

#define SIZE_ID     1
#define SIZE_TORQ   1
#define SIZE_LED    1
#define SIZE_GPOS   4
#define SIZE_PPOS   4
#define SIZE_VOLT   2

using namespace std;
using namespace dynamixel;
using namespace comm;
using namespace robot;

uint32_t float2pos(const float &deg)
{
    return static_cast<uint32_t>(deg/(360.0/(float)(MAX_POS-MIN_POS))+ZERO_POS);
}

motor::motor(const sub_ptr& s): timer(CONF.get_config_value<int>("serial.dxl.period")),sensor("motor"),
    period_(CONF.get_config_value<int>("serial.dxl.period")), p_count_(0)
{
    portHandler_ = PortHandler::getPortHandler(CONF.get_config_value<string>("serial.dxl.dev_name").c_str());
    packetHandler_ = PacketHandler::getPacketHandler(CONF.get_config_value<float>("serial.dxl.version"));
    ledWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_LED, SIZE_LED);
    torqWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_TORQ, SIZE_TORQ);
    gposWrite_ = make_shared<GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GPOS, SIZE_GPOS);
    dxl_error_ = 0;
    voltage_ = static_cast<uint16_t>(MAX_VOLTAGE*10);
    joint_degs_list.clear();
}

void motor::add_joint_degs(const map<int, float>& jdmap)
{
    lock_guard<mutex> lk(jd_mutex_);
    joint_degs_list.push_back(jdmap);
}

void motor::set_torq(uint8_t e)
{
    torqWrite_->clearParam();
    uint8_t torq_data;
    torq_data = e;
    for(auto r:ROBOT.get_joint_map())
        torqWrite_->addParam(static_cast<uint8_t>(r.second->jid_), &torq_data);
    torqWrite_->txPacket();
}

void motor::set_led(uint8_t s)
{
    ledWrite_->clearParam();
    uint8_t led_data;
    led_data = s;
    for(auto r:ROBOT.get_joint_map())
        ledWrite_->addParam(static_cast<uint8_t>(r.second->jid_), &led_data);
    ledWrite_->txPacket();
}

void motor::set_gpos()
{
    gposWrite_->clearParam();
    uint8_t gpos_data[4];
    uint32_t gpos;
    for(auto j:ROBOT.get_joint_map())
    {
        gpos = float2pos(j.second->get_deg());
        gpos_data[0] = DXL_LOBYTE(DXL_LOWORD(gpos));
        gpos_data[1] = DXL_HIBYTE(DXL_LOWORD(gpos));
        gpos_data[2] = DXL_LOBYTE(DXL_HIWORD(gpos));
        gpos_data[3] = DXL_HIBYTE(DXL_HIWORD(gpos));
        if(!gposWrite_->addParam(static_cast<uint8_t>(j.second->jid_), gpos_data)) return;
    }
    gposWrite_->txPacket();
}

bool motor::open()
{
    if(OPTS.use_robot())
    {
        if(portHandler_->openPort())
        {
            if(portHandler_->setBaudRate(CONF.get_config_value<int>("serial.dxl.baudrate")))
            {
                is_open_ = true;
                sensor::is_alive_ = true;
                timer::is_alive_ = true;
                return true;
            }
        }
    }
    if(OPTS.use_debug())
    {
        timer::is_alive_ = true;
        return true;
    }
    return false;
}

void motor::close()
{
    if(is_open_)
    {
        is_open_ = false;
        portHandler_->closePort();
    }
    sensor::is_alive_ = false;
    timer::is_alive_ = false;
    delete_timer();
    sleep(1);
    std::cout<<"\033[32mmotor closed!\033[0m\n";
}

bool motor::start()
{
    //cout<<"start\n";
    if(!this->open()) return false;
    if(OPTS.use_robot()) set_torq(1);
    start_timer();
    return true;
}

void motor::run()
{
    lock_guard<mutex> lk(jd_mutex_);
    std::map<int, float> jdmap;
    if(timer::is_alive_)
    {
        if(!joint_degs_list.empty())
        {
            jdmap = joint_degs_list.front();
            joint_degs_list.pop_front();
            ROBOT.set_degs(jdmap);
        }
        if(OPTS.use_debug())
        {
            tcp_packet::tcp_command cmd;
            cmd.type = tcp_packet::JOINT_DATA;
            robot_joint_deg jd;
            string j_data;
            j_data.clear();
            for(auto jm:ROBOT.get_joint_map())
            {
                jd.id = jm.second->jid_;
                jd.deg = jm.second->get_deg();
                j_data.append((char*)(&jd), sizeof(robot_joint_deg));
            }
            cmd.size = ROBOT.get_joint_map().size()*sizeof(robot_joint_deg);
            memcpy(cmd.data, j_data.c_str(), cmd.size);
            TCP_SERVER.write(cmd);
        }
        if(OPTS.use_robot())
        {
            set_gpos();
            if((p_count_*period_%1000) == 0)
            {
                led_status_ = 1 - led_status_;
                set_led(led_status_);
            }
            if((p_count_*period_%10000) == 0)
            {
                int res = packetHandler_->read2ByteTxRx(portHandler_, static_cast<uint8_t>(ROBOT.get_joint("jhead1")->jid_), 
                                                        ADDR_VOLT, (uint16_t*)&voltage_, &dxl_error_);
                if(res == COMM_SUCCESS) notify();
            }
        }
        p_count_++;
    }
}

motor::~motor()
{

}
