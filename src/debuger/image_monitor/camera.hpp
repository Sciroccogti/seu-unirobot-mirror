#ifndef CAMTHREAD_H
#define CAMTHREAD_H

#include <QtWidgets>
#include <opencv2/opencv.hpp>
using namespace cv;

class CameraDevice : public QObject
{
    Q_OBJECT
public:
    explicit CameraDevice(QObject *parent = 0);
    ~CameraDevice();
    Mat getSrcimage() const;
    bool start();
    bool stop();

signals:
    void imageReady(const Mat& image);

private slots:
    void onTimeout();

private:
    VideoCapture * m_capture;
    QTimer * m_timer;
    bool Camera_Init();
    Mat srcimage;

};

#endif // CAMTHREAD_H

