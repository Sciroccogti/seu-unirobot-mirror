#ifndef SEU_UNIROBOT_DEBUGER_CAMERADEVICE_HPP
#define SEU_UNIROBOT_DEBUGER_CAMERADEVICE_HPP

#include <QtWidgets>
#include <opencv2/opencv.hpp>

class CameraDevice: public QObject
{
    Q_OBJECT
public:
    explicit CameraDevice();
    bool start(const int &index=0);
    bool stop();
    cv::Mat getimage()
    {
        img_mutex.lock();
        cv::Mat res = srcimage;
        img_mutex.unlock();
        return res;
    }
signals:
    void imageReady();
private slots:
    void onTimeout();
private:
    bool Camera_Init(const int &index=0);
    cv::VideoCapture m_capture;
    QTimer *m_timer;
    cv::Mat srcimage;
    QMutex img_mutex;
};



#endif //SEU_UNIROBOT_CAMERADEVICE_HPP
