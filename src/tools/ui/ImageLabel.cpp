#include "ImageLabel.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

ImageLabel::ImageLabel(const int &w, const int &h)
{
    this->setFixedSize(w, h);
    this->setStyleSheet("QLabel{background:black}");
    drawing = false;
}

void ImageLabel::set_image(QImage im)
{
    image = QImage(im);
    this->repaint();
}

void ImageLabel::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        drawing = true;
        startPoint = event->pos();
    }
}

void ImageLabel::mouseMoveEvent(QMouseEvent *event)
{
    if (drawing)
    {
        endPoint.setX(event->pos().x());
        endPoint.setY(event->pos().y());
        shotRect = QRect(startPoint, endPoint);
        this->update();
    }
}

void ImageLabel::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        shotRect = QRect(startPoint, endPoint);
        emit shot(shotRect);
        drawing = false;
        this->update();
    }
}

void ImageLabel::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);

    if (!image.isNull())
    {
        QPixmap pixmap = QPixmap::fromImage(image);
        painter.drawPixmap(0, 0, image.width(), image.height(), pixmap);

        if (drawing)
        {
            painter.drawRect(shotRect);
        }
    }
}

