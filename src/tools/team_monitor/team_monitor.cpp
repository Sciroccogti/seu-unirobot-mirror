#include "team_monitor.hpp"
#include "configuration.hpp"
#include "parser/field_parser.hpp"

using boost::asio::ip::udp;
using namespace std;

boost::asio::io_service udp_service;

team_monitor::team_monitor(): socket_(udp_service, udp::endpoint(udp::v4(), CONF->get_config_value<short>("net.udp.team.port")))
{
    parser::field_parser::parse(CONF->field_file(), field_);
    setFixedSize(field_.field_length + 2 * field_.border_strip_width_min, field_.field_width + 2 * field_.border_strip_width_min);
    setStyleSheet("background:green");
    td_ = std::move(std::thread([this]()
    {
        this->receive();
        udp_service.run();
    }));
}

team_monitor::~team_monitor()
{
    if(td_.joinable())
    {
        td_.join();
    }
}

void team_monitor::receive()
{
    socket_.async_receive_from(boost::asio::buffer((char *)&pkt_, sizeof(comm_packet)), point_,
                               [this](boost::system::error_code ec, std::size_t bytes_recvd)
    {
        if (!ec && bytes_recvd > 0)
        {
            string recv_header;
            recv_header.append(pkt_.header, sizeof(comm_packet::header));
            if(recv_header==COMM_DATA_HEADER)
            {
                p_mutex_.lock();
                players_[pkt_.info.id] = pkt_.info;
                p_mutex_.unlock();
                update();
            }
        }

        receive();
    });
}

void team_monitor::closeEvent(QCloseEvent *event)
{
    socket_.cancel();
    udp_service.stop();
}

void team_monitor::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.translate(field_.field_length / 2 + field_.border_strip_width_min, field_.field_width / 2 + field_.border_strip_width_min);
    painter.setPen(QPen(Qt::white, 4, Qt::SolidLine, Qt::FlatCap));
    painter.drawEllipse(-field_.center_circle_diameter / 2, -field_.center_circle_diameter / 2, field_.center_circle_diameter, field_.center_circle_diameter);
    painter.drawLine(0, -field_.field_width / 2, 0, field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, -field_.field_width / 2, field_.field_length / 2, -field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, -field_.field_width / 2, -field_.field_length / 2, field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, field_.field_width / 2, field_.field_length / 2, field_.field_width / 2);
    painter.drawLine(field_.field_length / 2, -field_.field_width / 2, field_.field_length / 2, field_.field_width / 2);
    painter.drawLine(-field_.field_length / 2, -field_.goal_area_width / 2, -(field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2);
    painter.drawLine(-field_.field_length / 2, field_.goal_area_width / 2, -(field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine(-(field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2, -(field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine(-(field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, -field_.field_length / 2, -field_.goal_width / 2);
    painter.drawLine(-(field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2, -field_.field_length / 2, field_.goal_width / 2);
    painter.drawLine(-(field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, -(field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2);
    painter.drawLine(field_.field_length / 2, -field_.goal_area_width / 2, (field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2);
    painter.drawLine(field_.field_length / 2, field_.goal_area_width / 2, (field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine((field_.field_length / 2 - field_.goal_area_length), -field_.goal_area_width / 2, (field_.field_length / 2 - field_.goal_area_length), field_.goal_area_width / 2);
    painter.drawLine((field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, field_.field_length / 2, -field_.goal_width / 2);
    painter.drawLine((field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2, field_.field_length / 2, field_.goal_width / 2);
    painter.drawLine((field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, (field_.field_length / 2 + field_.goal_depth), field_.goal_width / 2);

    painter.setPen(QPen(Qt::white, 2, Qt::SolidLine, Qt::FlatCap));
    painter.setBrush(QBrush(Qt::white, Qt::NoBrush));
    painter.drawRect(-(field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, field_.goal_depth, field_.goal_width);
    painter.drawRect((field_.field_length / 2 + field_.goal_depth), -field_.goal_width / 2, -field_.goal_depth, field_.goal_width);
    painter.setPen(QPen(Qt::white, 1, Qt::SolidLine, Qt::FlatCap));
    painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
    painter.drawRect(-4, -4, 8, 8);
    painter.drawRect((field_.field_length / 2 - field_.penalty_mark_distance) - 4, -4, 8, 8);
    painter.drawRect(-(field_.field_length / 2 - field_.penalty_mark_distance) + 4, -4, 8, 8);

    int ballsize = 20;
    painter.setBrush(QBrush(Qt::black, Qt::SolidPattern));
    p_mutex_.lock();

    for (auto &p : players_)
    {
        painter.setPen(QPen(Qt::white, 2, Qt::SolidLine, Qt::FlatCap));
        painter.save();
        painter.translate(p.second.y * 100, p.second.x * 100);
        painter.drawEllipse(-ballsize / 2, -ballsize / 2, ballsize, ballsize);
        painter.drawText(-ballsize / 2, -ballsize / 2, QString::number(p.second.id));
        painter.rotate(-p.second.dir);
        painter.drawLine(0, 0, 2 * ballsize, 0);
        painter.restore();
        if(p.second.available)
        {
            painter.drawEllipse(p.second.ball_y * 100 - ballsize / 2, p.second.ball_x * 100 - ballsize / 2, ballsize, ballsize);
            painter.drawText(p.second.ball_y * 100 - ballsize / 2, p.second.ball_x * 100 - ballsize / 2, QString::number(p.second.id));
        }
    }
    p_mutex_.unlock();
}

void team_monitor::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
    {
        case Qt::Key_C:
            p_mutex_.lock();
            players_.clear();
            p_mutex_.unlock();
            break;
        default:
            break;
    }
    this->update();
}