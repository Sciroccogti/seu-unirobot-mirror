#include "image_debuger.hpp"
#include "configuration.hpp"
#include "ui/walk_remote.hpp"
#include "ui/camera_setter.hpp"
#include <opencv2/opencv.hpp>
#include "image/image_process.hpp"

using namespace cv;
using namespace std;
using namespace robot;
using namespace image;

image_debuger::image_debuger()
{
    height_ = CONF->get_config_value<int>("video.height");
    width_ = CONF->get_config_value<int>("video.width");
   
    srcLab = new ImageLabel(width_, height_);
    dstLab = new ImageLabel(width_, height_);
    curr_image_.create(height_, width_, CV_8UC3);
    curr_index_ = 0;
    infoLab = new QLabel("0/0");
    statusBar()->addWidget(infoLab);
    
    QHBoxLayout *imageLayout = new QHBoxLayout;
    imageLayout->addWidget(srcLab);
    imageLayout->addWidget(dstLab);
    
    btnLoad = new QPushButton("Load File");
    btnLast = new QPushButton("Last Frame");
    btnNext = new QPushButton("Next Frame");
    boxAuto = new QCheckBox("Auto Play(ms)");
    boxAuto->setFixedWidth(120);
    delayEdit = new QLineEdit("1000");
    delayEdit->setFixedWidth(50);
    QHBoxLayout *ctrlLayout = new QHBoxLayout;
    ctrlLayout->addWidget(btnLoad);
    ctrlLayout->addWidget(btnLast);
    ctrlLayout->addWidget(btnNext);
    ctrlLayout->addWidget(boxAuto);
    ctrlLayout->addWidget(delayEdit);
    
    frmSlider = new QSlider(Qt::Horizontal);
    frmSlider->setEnabled(false);
    
    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->addLayout(imageLayout);
    mainLayout->addLayout(ctrlLayout);
    mainLayout->addWidget(frmSlider);
    
    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    timer= new QTimer;
    connect(timer, &QTimer::timeout, this, &image_debuger::procTimer);
    connect(btnLoad, &QPushButton::clicked, this, &image_debuger::procBtnLoad);
    connect(btnLast, &QPushButton::clicked, this, &image_debuger::procBtnLast);
    connect(btnNext, &QPushButton::clicked, this, &image_debuger::procBtnNext);
    connect(boxAuto, &QCheckBox::stateChanged, this, &image_debuger::procBoxAuto);
    connect(frmSlider, &QSlider::valueChanged, this, &image_debuger::procFrmSlider);
}

void image_debuger::proc_image(const int& index)
{
    if(index<1 || index>yuv_images_.size()) return;
    curr_index_ = index;
    infoLab->setText(QString::number(curr_index_)+"/"+QString::number(yuv_images_.size()));
    frmSlider->setValue(index);
    Mat src, dst;
    cvtColor(yuv_images_[curr_index_-1], src, CV_YUV2RGB);  
    QImage *srcImage = new QImage((const unsigned char*)(src.data),src.cols,src.rows,QImage::Format_RGB888);
    srcLab->setPixmap(QPixmap::fromImage(srcImage->scaled(srcLab->size(), Qt::KeepAspectRatio)));
    
    cvtColor(src, dst, CV_RGB2BGR);
    QImage *dstImage = new QImage((const unsigned char*)(dst.data),dst.cols,dst.rows,
                                  dst.channels()==3?QImage::Format_RGB888:QImage::Format_Grayscale8);
    dstLab->setPixmap(QPixmap::fromImage(dstImage->scaled(dstLab->size(), Qt::KeepAspectRatio)));
    delete srcImage;
    delete dstImage;
}

void image_debuger::procBtnLast()
{
    curr_index_--;
    if(curr_index_<1) 
        curr_index_ = yuv_images_.size();
    proc_image(curr_index_);
}

void image_debuger::procBtnNext()
{
    curr_index_++;
    if(curr_index_>yuv_images_.size()) 
        curr_index_ = 1;
    proc_image(curr_index_);
}

void image_debuger::procBtnLoad()
{
    timer->stop();
    QString filename = QFileDialog::getOpenFileName(this, "Open file", QDir::homePath(), tr("*.yuv"));
    if(filename.isEmpty()) return;
    yuv_images_.clear();
    yuv_images_ = image_process::read_yuv(filename.toStdString());
    if(!yuv_images_.empty())
    {
        frmSlider->setEnabled(true);
        frmSlider->setMinimum(1);
        frmSlider->setMaximum(yuv_images_.size());
        proc_image(1);
    }
}

void image_debuger::procBoxAuto()
{
    if(boxAuto->checkState()== Qt::Checked)
    {
        int delay = delayEdit->text().toInt();
        if(delay<10) delay = 10;
        timer->start(delay);
    }
    else
        timer->stop();
}

void image_debuger::procFrmSlider(int v)
{
    proc_image(v);
}


void image_debuger::procTimer()
{
    procBtnNext();
}
