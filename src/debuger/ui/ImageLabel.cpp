#include "ImageLabel.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

ImageLabel::ImageLabel(const int& w, const int& h)
{
    this->setFixedSize(w,h);
    this->setStyleSheet("QLabel{background:black}");
    pixmap = new QPixmap(w,h);
}

void ImageLabel::set_image(unsigned char* src, const int size)
{
    vector<unsigned char> buf(size);
    memcpy(&buf[0], src, size);
    Mat bgr = imdecode(buf, cv::IMREAD_COLOR);
    Mat dst;
    cvtColor(bgr, dst, CV_BGR2RGB);
    pixmap->loadFromData(dst.data, dst.rows*dst.cols*dst.channels());
    this->setPixmap(*pixmap);
    bgr.release();
    dst.release();
    update();
}