#include "ImageLabel.hpp"

ImageLabel::ImageLabel(const int& w, const int& h)
{
    this->setFixedSize(w,h);
    this->setStyleSheet("QLabel{background:black}");
}

void ImageLabel::set_image(unsigned char* src, const int& size)
{
    image_.loadFromData((unsigned char*)src, size);
     update();
}

