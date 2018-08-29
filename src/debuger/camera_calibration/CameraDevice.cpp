#include "CameraDevice.hpp"

using namespace cv;

CameraDevice::CameraDevice()
{
    m_timer = new QTimer(this);
    m_timer->start(40);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
}

bool CameraDevice::start(const int &index)
{
    if (m_capture.isOpened())
        return true;
    if(!Camera_Init(index))
    {
        QMessageBox::critical(NULL,"error","Open camera error!",QMessageBox::NoButton,QMessageBox::NoButton);
        return false;
    }
    return m_capture.isOpened();
}

bool CameraDevice::stop()
{
    if(m_capture.isOpened())
        m_capture.release();
    return true;
}

void CameraDevice::onTimeout()
{
    if(!m_capture.isOpened()) return;
    img_mutex.lock();
    m_capture >> srcimage;
    img_mutex.unlock();
    if (!srcimage.empty())
        emit imageReady();
}

bool CameraDevice::Camera_Init(const int &index)
{
    m_capture.open(index);
    if (m_capture.isOpened())
    {
        m_capture.set(CV_CAP_PROP_FPS,25);
        return true;
    }
    return false;
}
