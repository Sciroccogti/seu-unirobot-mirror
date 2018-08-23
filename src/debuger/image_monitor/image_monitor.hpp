#ifndef SEU_UNIROBOT_DEBUGER_ACTION_MONITOR_HPP
#define SEU_UNIROBOT_DEBUGER_ACTION_MONITOR_HPP

#include <QtWidgets>
#include "robot/humanoid.hpp"
#include "tcp_client/tcp_client.hpp"
#include "ui/ImageLabel.hpp"
#include <opencv2/opencv.hpp>

class image_monitor: public QMainWindow
{
    Q_OBJECT
public:
    image_monitor();
    void data_handler(const tcp_command cmd);
public slots:
    void procTimer();
    void procYawSlider(int v);
    void procPitchSlider(int v);
    void procBtnWR();

protected:
    void closeEvent(QCloseEvent *event);
private:
    QPushButton *btnWR;
    ImageLabel *imageLab;
    QLabel *yawLab, *pitchLab, *netLab;
    QSlider *pitchSlider, *yawSlider;
    QTimer *timer;
    tcp_client client_;
    QString net_info;
    bool first_connect;
    cv::Mat curr_image_;
    int width_;
    int height_;
};

#endif //SEU_UNIROBOT_ACTION_MONITOR_HPP
