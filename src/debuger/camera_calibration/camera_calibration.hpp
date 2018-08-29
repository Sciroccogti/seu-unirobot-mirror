#ifndef SEU_UNIROBOT_DEBUGER_CAMERA_CALIBRATION_HPP
#define SEU_UNIROBOT_DEBUGER_CAMERA_CALIBRATION_HPP

#include <QtWidgets>
#include "CameraDevice.hpp"
class QInfoEdit: public QWidget
{
public:
    QInfoEdit(const QString &name)
    {
        QLabel *nameLab = new QLabel(name);
        nameLab->setFixedWidth(120);
        edit = new QLineEdit();
        QHBoxLayout *mainLayout = new QHBoxLayout();
        mainLayout->addWidget(nameLab);
        mainLayout->addWidget(edit);
        setLayout(mainLayout);
    }
    QLineEdit *edit;
};

class camera_calibration: public QMainWindow
{
public:
    camera_calibration();

protected:
    void procImageReady();
    void procBtnStart();
    void procBtnStop();
    void procBtnPhoto();
    void procBtnCali();
    void closeEvent(QCloseEvent *event);
private:
    QString dirname;
    QLabel *imgLab, *statusLab;
    CameraDevice *camera;
    QThread *camThread;
    QPushButton *startBtn, *stopBtn, *photoBtn;
    QPushButton *caliBtn;
    QInfoEdit *camIndex;
    QInfoEdit *squareEdit;
    QInfoEdit *boardCornerW, *boardCornerH;
    int pic_index;
};


#endif
