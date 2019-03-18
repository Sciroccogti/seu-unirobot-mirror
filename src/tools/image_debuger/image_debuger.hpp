#pragma once

#include <QtWidgets>
#include "robot/humanoid.hpp"
#include "ui/ImageLabel.hpp"
#include <opencv2/opencv.hpp>
#include "darknet/network.h"
#include "model.hpp"

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
    void procShot(QRect rect);
    void procFrmSlider(int v);
private:
    void proc_image(const unsigned int &index);
    void show_src();
    void show_dst(cv::Mat dst);

    QPushButton *btnLoad, *btnNext, *btnLast;
    QCheckBox *boxAuto;
    ImageLabel *srcLab, *dstLab;
    QLabel *infoLab;
    QTimer *timer;
    QSlider *frmSlider;
    QLineEdit *delayEdit;
    QComboBox *funcBox;
    unsigned int curr_index_;
    cv::Mat curr_image_;
    QString curr_dir_;
    QStringList image_names_;
    cv::Mat rgb_src_;
    int width_;
    int height_;
    network net_;
    object_prob ball_, post_;
};
