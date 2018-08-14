#include "ImageLabel.hpp"

ImageLabel::ImageLabel(const int& w, const int& h)
{
    this->setFixedSize(w,h);
    this->setStyleSheet("QLabel{background:black}");
}

void ImageLabel::set_image(unsigned char* src, const int& size)
{
    image_.loadFromData((unsigned char*)src, size);
}

void ImageLabel::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    QPixmap *pixmap = new QPixmap(image_.size());
    *pixmap = QPixmap::fromImage(image_);
    painter.drawPixmap(0, 0, this->width(), this->height(), *pixmap);
    delete pixmap;
}
