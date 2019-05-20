#ifndef __IMAGE_MONITOR_HPP
#define __IMAGE_MONITOR_HPP

#include <QtWidgets>
#include <atomic>
#include "robot/robot.hpp"
#include "tcp_client/tcp_client.hpp"
#include "ui/image_label.hpp"
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

class DisDialog: public QDialog
{
public:
    DisDialog()
    {
        xEdit = new QLineEdit("x real");
        yEdit = new QLineEdit("y real");
        okBtn = new QPushButton("OK");
        QHBoxLayout *layout = new QHBoxLayout();
        layout->addWidget(xEdit);
        layout->addWidget(yEdit);
        layout->addWidget(okBtn);
        setLayout(layout);
        connect(okBtn, &QPushButton::clicked, this, &DisDialog::procBtnOK);
    }

    Eigen::Vector2f get_real_dis()
    {
        bool ok1, ok2;
        float x = xEdit->text().toFloat(&ok1);
        float y = yEdit->text().toFloat(&ok2);
        return (ok1&&ok2)?Eigen::Vector2f(x,y):Eigen::Vector2f(-1, -1);
    }

    void procBtnOK()
    {
        close();
    }
private:
    QLineEdit *xEdit, *yEdit;
    QPushButton *okBtn;
};

class ImageMonitor: public QMainWindow
{
    Q_OBJECT
public:
    ImageMonitor();
    void data_handler(const tcp_command cmd);
public slots:
    void procTimer();
    void procYawSlider(int v);
    void procPitchSlider(int v);
    void procBtnWR();
    void procBtnCS();
    void procShot(QRect rect);
    void procImageBox(int idx);
    void procDisRecved(float x, float y);
signals:
    void disRecved(float, float);

protected:
    void closeEvent(QCloseEvent *event);
    void keyPressEvent(QKeyEvent *event);
private:
    QPushButton *btnWR, *btnCS;
    ImageLabel *imageLab;
    QLabel *yawLab, *pitchLab, *netLab;
    QSlider *pitchSlider, *yawSlider;
    QComboBox *sendBox;
    QComboBox *imageBox;
    QCheckBox *sampleBox;
    QTimer *timer;
    tcp_client client_;
    QString net_info;
    bool first_connect;
    cv::Mat curr_image_;
    int width_;
    int height_;
    unsigned int image_count_;
    std::atomic_bool save_;
};

#endif
