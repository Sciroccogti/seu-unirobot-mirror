#include "image_marker.hpp"
#include "configuration.hpp"
#include "ui/walk_remote.hpp"
#include "ui/camera_setter.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace cv;
using namespace std;

ImgLabel::ImgLabel(const int &w, const int &h)
{
    this->setFixedSize(w, h);
    this->setStyleSheet("QLabel{background:black}");
    choose = false;
}


void ImgLabel::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        choose = true;
        startPoint = event->pos();
    }
}

void ImgLabel::mouseMoveEvent(QMouseEvent *event)
{
    if (choose)
    {
        endPoint.setX(event->pos().x());
        endPoint.setY(event->pos().y());
        shotRect = QRect(startPoint, endPoint);
        emit shot(shotRect, false);
    }
}

void ImgLabel::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        shotRect = QRect(startPoint, endPoint);
        emit shot(shotRect, true);
        choose = false;
    }
}


image_marker::image_marker()
{
    imageLab = new ImgLabel();
    infoLab = new QLabel("0/0");
    
    QVBoxLayout *leftLayout = new QVBoxLayout();
    leftLayout->addWidget(imageLab);

    btnOpenImageFolder = new QPushButton("Choose Image Folder");
    btnOpenNameFile = new QPushButton("Choose Names File");\
    btnSave = new QPushButton("Save");
    btnClear = new QPushButton("Clear");
    classesBox = new QComboBox();
    QVBoxLayout *ctrlLayout = new QVBoxLayout;
    ctrlLayout->addWidget(btnOpenImageFolder);
    ctrlLayout->addWidget(btnOpenNameFile);
    ctrlLayout->addWidget(classesBox);
    ctrlLayout->addWidget(btnClear);
    ctrlLayout->addWidget(btnSave);

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(ctrlLayout);

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    curr_id_ = 0;
    statusBar()->addWidget(infoLab);
    connect(btnOpenImageFolder, &QPushButton::clicked, this, &image_marker::procBtnIF);
    connect(btnOpenNameFile, &QPushButton::clicked, this, &image_marker::procBtnNF);
    connect(imageLab, &ImgLabel::shot, this, &image_marker::procShot);
    connect(btnClear, &QPushButton::clicked, this, &image_marker::procBtnCL);
    connect(btnSave, &QPushButton::clicked, this, &image_marker::procBtnSV);
}

void image_marker::procBtnIF()
{
    QString foldername = QFileDialog::getExistingDirectory(this, "Open Image Folder", QDir::homePath());
    if(!foldername.isEmpty())
    {
        QString image_folder_name = foldername + "/";
        image_folder_ = image_folder_name;
        QDir dir(image_folder_name); 
        QStringList nameFilters; 
        nameFilters << "*.jpg"; 
        nameFilters << "*.png";
        QStringList image_file_names = dir.entryList(nameFilters, QDir::Files|QDir::Readable, QDir::Name); 
        curr_id_ = 0;
        infoLab->setText(QString::number(curr_id_)+"/"+QString::number(image_file_names.size()));
        sample_infos.clear();
        for(int i=0;i<image_file_names.size();i++)
            sample_infos.push_back(ImageSampleInfo(image_folder_name.toStdString()+image_file_names[i].toStdString()));
    }
}

void image_marker::procBtnNF()
{
    QString filename = QFileDialog::getOpenFileName(this, "Open Names File", QDir::homePath(), tr("*.names"));
    if(!filename.isEmpty())
    {
        names_filename_ = filename;
        classesBox->clear();
        ifstream nf(filename.toStdString().c_str());
        if(!nf) return;
        string s;
        while(getline(nf, s))
        {
            classesBox->addItem(QString::fromStdString(s));
        }
        nf.close();
        update();
    }
}

void image_marker::procShot(QRect rect, bool finish)
{
    int x, y, w, h;
    x = rect.left();
    y = rect.top();
    w = rect.width();
    h = rect.height();
    if(w<10||h<10) return;
    if(classesBox->count()==0)
    {
        QMessageBox::warning(this, "Warning", "No classes!");
        return;
    }
    float dw = 1.0/curr_image_.size().width;
    float dh = 1.0/curr_image_.size().height;
    Mat tmp;
    curr_image_.copyTo(tmp);
    rectangle(tmp, Point(x,y), Point(x+w, y+h), Scalar(255, 0, 0, 0));
    putText(tmp, String(classesBox->currentText().toStdString().c_str()), Point(x, y),
            FONT_HERSHEY_SIMPLEX,1,Scalar(0, 0,255, 0));
    show_image(tmp);
    
    if(finish)
        sample_infos[curr_id_-1].samples.push_back(ObjInfo(classesBox->currentIndex(), rect.left()*dw, rect.top()*dh,
            rect.width()*dw, rect.height()*dh));
}

void image_marker::procBtnCL()
{
    sample_infos[curr_id_-1].samples.clear();
    Mat tmp;
    curr_image_.copyTo(tmp);
    show_image(tmp);
}

void image_marker::procBtnSV()
{
    ofstream img_file("train.txt");
    for(ImageSampleInfo sp:sample_infos)
    {
        string tname = sp.image_name.substr(0, sp.image_name.find_last_of('.'))+".txt";
        img_file<<sp.image_name<<endl;
        ofstream ofs(tname);
        for(ObjInfo obj:sp.samples)
        {
            float cx = (obj.x+obj.w)/2.0;
            float cy = (obj.y+obj.h)/2.0;
            ofs<<obj.id<<" "<<cx<<" "<<cy<<" "<<obj.w<<" "<<obj.h<<endl;
        }
        ofs.close();
    }
    img_file.close();
    QMessageBox::information(this, "information", "Save success!");
}

void image_marker::show_image(Mat img)
{
    Mat rgb;
    if(img.empty()) return;
    int x, y, w, h;
    float dw = 1.0/curr_image_.size().width;
    float dh = 1.0/curr_image_.size().height;
    ImageSampleInfo &sp = sample_infos[curr_id_-1];
    for(int i=0;i<sp.samples.size();i++)
    {
        x = sp.samples[i].x/dw;
        y = sp.samples[i].y/dh;
        w = sp.samples[i].w/dw;
        h = sp.samples[i].h/dh;
        rectangle(img, Point(x,y), Point(x+w, y+h), Scalar(i*20%255, i*80%255,i*160%255, 0), 3);
        putText(img, String(classesBox->itemText(sp.samples[i].id).toStdString().c_str()), Point(x, y),
            FONT_HERSHEY_SIMPLEX,1,Scalar(i*20%255, i*80%255,i*160%255, 0), 3);
    }
    cvtColor(img, rgb, CV_BGR2RGB);
    imageLab->setFixedSize(rgb.size().width,rgb.size().height);
    QImage srcImage(rgb.data, rgb.cols, rgb.rows, QImage::Format_RGB888);
    imageLab->setPixmap(QPixmap::fromImage(srcImage));
}

void image_marker::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "write action data into file?",
                                        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);

    if (reply == QMessageBox::StandardButton::Yes)
    {
        procBtnSV();
    }
}

void image_marker::keyPressEvent(QKeyEvent* event)
{
    switch (event->key())
    {
        case Qt::Key_Left:
            curr_id_--;
            break;
        case Qt::Key_Right:
            curr_id_++;
            break;
        default:
            break;
    }
    if(sample_infos.size()>0)
    {
        if(curr_id_<1) curr_id_=sample_infos.size();
        else if(curr_id_>sample_infos.size()) curr_id_ = 1;
        infoLab->setText(QString::number(curr_id_)+"/"+QString::number(sample_infos.size()));
        curr_image_ = imread(String(sample_infos[curr_id_-1].image_name.c_str()));
        Mat tmp;
        curr_image_.copyTo(tmp);
        show_image(tmp);
    }
    else curr_id_ = 0;
}

