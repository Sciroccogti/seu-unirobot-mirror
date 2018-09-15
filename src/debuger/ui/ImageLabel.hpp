#ifndef SEU_UNIROBOT_DEBUGER_IMAGELABLE_HPP
#define SEU_UNIROBOT_DEBUGER_IMAGELABLE_HPP

#include <QtWidgets>

class ImageLabel: public QLabel
{
    Q_OBJECT
public:
    ImageLabel(const int &w = 640, const int &h = 480);
};

#endif