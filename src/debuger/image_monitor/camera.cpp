#include "camera.hpp"

CameraDevice::CameraDevice(QObject *parent)
    :QObject(parent)
{
    qRegisterMetaType<Mat>("Mat");
    m_capture = new VideoCapture;
    m_timer = new QTimer(this);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    start();
}

CameraDevice::~CameraDevice()
{
    delete m_capture;
    m_capture = NULL;
}

bool CameraDevice::start()
{
    if (m_capture->isOpened())
        return true;
    if(!Camera_Init())
    {
        QMessageBox::critical(NULL,"error","Open camera error!",QMessageBox::NoButton,QMessageBox::NoButton);
        exit(1);
    }
    if (m_capture->isOpened())
        m_timer->start(40);
    return m_capture->isOpened();
}

bool CameraDevice::stop()
{
    if(m_capture->isOpened())
        m_capture->release();
    return true;
}

void CameraDevice::onTimeout()
{
    if(!m_capture->isOpened())
        return;
    static Mat frame;
    *m_capture >> frame;
    if (!frame.empty())
    {
        emit imageReady(frame);
    }
}

bool CameraDevice::Camera_Init()
{
    m_capture->open(0);
    return true;
}

Mat CameraDevice::getSrcimage() const
{
    return srcimage;
}

