#pragma once

#include <QtWidgets>
#include <opencv2/opencv.hpp>

struct ObjInfo
{
    int id;
    float x, y;
    float w, h;
    ObjInfo(int i, float xx, float yy, float ww, float hh): id(i), x(xx), y(yy), w(ww), h(hh) {}
};

struct ImageSampleInfo
{
    std::string image_name;
    std::vector<ObjInfo> samples;
    ImageSampleInfo(std::string name): image_name(name) {}
};

class ImgLabel: public QLabel
{
    Q_OBJECT
public:
    ImgLabel(const int &w = 640, const int &h = 480);
protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

signals:
    void shot(QRect, bool);
private:
    bool choose;
    QPoint startPoint;
    QPoint endPoint;
    QRect shotRect;
};

class image_marker: public QMainWindow
{
    Q_OBJECT
public:
    image_marker();
public slots:
    void procBtnIF();
    void procBtnNF();
    void procBtnSV();
    void procBtnCL();
    void procShot(QRect rect, bool finish);

protected:
    void closeEvent(QCloseEvent *event);
    void keyPressEvent(QKeyEvent *event);
private:
    void show_image(cv::Mat img);

    QPushButton *btnOpenImageFolder, *btnOpenNameFile;
    QPushButton *btnSave, *btnClear;
    ImgLabel *imageLab;
    QLabel *infoLab;
    QComboBox *classesBox;
    QString names_filename_, image_folder_;
    cv::Mat curr_image_;
    std::vector<ImageSampleInfo> sample_infos;

    int curr_id_;
    int width_;
    int height_;
};
