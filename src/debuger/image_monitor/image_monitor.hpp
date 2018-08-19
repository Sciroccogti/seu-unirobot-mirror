#ifndef SEU_UNIROBOT_DEBUGER_ACTION_MONITOR_HPP
#define SEU_UNIROBOT_DEBUGER_ACTION_MONITOR_HPP

#include <QtWidgets>
#include "robot/humanoid.hpp"
#include "tcp_client/tcp_client_handler.hpp"
#include "module/ImageLabel.hpp"
#include "camera.hpp"

class image_monitor: public QMainWindow
{
    Q_OBJECT
public:
    image_monitor();
    void data_handler(const char *data, const int &size, const int &type);
public slots:
    void procTimer();
    void procYawSlider(int v);
    void procPitchSlider(int v);
    void srcImageReady(Mat);

protected:
    void closeEvent(QCloseEvent *event);
private:
    ImageLabel *imageLab;
    QLabel *yawLab, *pitchLab, *netLab;
    QSlider *pitchSlider, *yawSlider;
    QTimer *timer;
    tcp_client_handler client_;
    QString net_info;
    bool first_connect;
    CameraDevice *cdevice;
    QThread *cthread;
};

#endif //SEU_UNIROBOT_ACTION_MONITOR_HPP
