#ifndef SEU_UNIROBOT_DEBUGER_IMAGE_DEBUGER_HPP
#define SEU_UNIROBOT_DEBUGER_IMAGE_DEBUGER_HPP

#include <QtWidgets>
#include "robot/humanoid.hpp"
#include "tcp_client/tcp_client.hpp"
#include "ui/ImageLabel.hpp"
#include <opencv2/opencv.hpp>

class image_debuger: public QMainWindow
{
    Q_OBJECT
public:
    image_debuger();
public slots:
    void procTimer();
    void procBtnLoad();
    void procBtnLast();
    void procBtnNext();
    void procBoxAuto();
    void procFrmSlider(int v);
private:
    void proc_image(const unsigned int &index);
    QPushButton *btnLoad, *btnNext, *btnLast;
    QCheckBox *boxAuto;
    ImageLabel *srcLab, *dstLab;
    QLabel *infoLab;
    QTimer *timer;
    QSlider *frmSlider;
    QLineEdit *delayEdit;
    unsigned int curr_index_;
    cv::Mat curr_image_;
    std::vector<cv::Mat> yuv_images_;
    int width_;
    int height_;
};

#endif
