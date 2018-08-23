#include "ImageLabel.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

ImageLabel::ImageLabel(const int& w, const int& h)
{
    this->setFixedSize(w,h);
    this->setStyleSheet("QLabel{background:black}");
}