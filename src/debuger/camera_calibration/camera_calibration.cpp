#include "camera_calibration.hpp"
#include <fstream>

using namespace std;
using namespace cv;

camera_calibration::camera_calibration()
{
    pic_index = 0;
    dirname = "image/";
    imgLab = new QLabel();
    imgLab->setFixedSize(640, 480);
    imgLab->setStyleSheet("QLabel{background:black}");
    statusLab = new QLabel();
    statusBar()->addWidget(statusLab);
    setStyleSheet("QGroupBox{border: 1px solid}");

    camIndex = new QInfoEdit("Camera Index");
    startBtn = new QPushButton("Start Camera");
    stopBtn = new QPushButton("Stop Camera");
    photoBtn = new QPushButton("Take Photo");
    stopBtn->setEnabled(false);
    photoBtn->setEnabled(false);

    squareEdit = new QInfoEdit("Square Size(mm)");
    boardCornerW = new QInfoEdit("X Inner Corner");
    boardCornerH = new QInfoEdit("Y Inner Corner");
    caliBtn = new QPushButton("Calibration");
    caliBtn->setEnabled(false);

    QVBoxLayout *camLayout = new QVBoxLayout();
    camLayout->addWidget(camIndex);
    camLayout->addWidget(startBtn);
    camLayout->addWidget(stopBtn);
    camLayout->addWidget(photoBtn);

    QVBoxLayout *caliLayout = new QVBoxLayout();
    caliLayout->addWidget(squareEdit);
    caliLayout->addWidget(boardCornerW);
    caliLayout->addWidget(boardCornerH);
    caliLayout->addWidget(caliBtn);

    QGroupBox *camGroup = new QGroupBox("Camera");
    camGroup->setFixedHeight(200);
    camGroup->setLayout(camLayout);
    QGroupBox *caliGroup = new QGroupBox("Calibration");
    caliGroup->setLayout(caliLayout);

    QVBoxLayout *rightLayout = new QVBoxLayout();
    rightLayout->addWidget(camGroup);
    rightLayout->addWidget(caliGroup);

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addWidget(imgLab);
    mainLayout->addLayout(rightLayout);

    QWidget *mainWidget = new QWidget();
    mainWidget->setLayout(mainLayout);
    setCentralWidget(mainWidget);

    camera = new CameraDevice();

    connect(camera, &CameraDevice::imageReady, this, &camera_calibration::procImageReady);
    connect(startBtn, &QPushButton::clicked, this, &camera_calibration::procBtnStart);
    connect(stopBtn, &QPushButton::clicked, this, &camera_calibration::procBtnStop);
    connect(photoBtn, &QPushButton::clicked, this, &camera_calibration::procBtnPhoto);
    connect(caliBtn, &QPushButton::clicked, this, &camera_calibration::procBtnCali);

    camThread = new QThread(this);
    camera->moveToThread(camThread);
    camThread->start(QThread::HighPriority);
}

void camera_calibration::procImageReady()
{
    Mat dst;
    cvtColor(camera->getimage(), dst, CV_BGR2RGB);
    QImage *disImage = new QImage((const unsigned char *)(dst.data), dst.cols, dst.rows, QImage::Format_RGB888);
    imgLab->setPixmap(QPixmap::fromImage(disImage->scaled(imgLab->size(), Qt::KeepAspectRatio)));
}

void camera_calibration::procBtnStart()
{
    if (!camera->start(camIndex->edit->text().toInt()))
    {
        QMessageBox::warning(this, "Error", "Open camera orror!");
    }
    else
    {
        startBtn->setEnabled(false);
        stopBtn->setEnabled(true);
        photoBtn->setEnabled(true);
        camIndex->setEnabled(false);
    }
}

void camera_calibration::procBtnStop()
{
    camera->stop();
    startBtn->setEnabled(true);
    stopBtn->setEnabled(false);
    photoBtn->setEnabled(false);
    camIndex->setEnabled(true);
}

void camera_calibration::procBtnPhoto()
{
    QDir dir;

    if (!dir.exists(dirname))
    {
        dir.mkdir(dirname);
    }

    QString picname = dirname + QString::number(pic_index++) + ".png";
    statusLab->setText(picname);
    imwrite(String(picname.toStdString().c_str()), camera->getimage());

    if (pic_index > 10)
    {
        caliBtn->setEnabled(true);
    }
}

void camera_calibration::procBtnCali()
{
    Mat cameraMatrix, distCoeffs;
    Size boardSize, imageSize;
    boardSize.width = boardCornerW->edit->text().toInt();
    boardSize.height = boardCornerH->edit->text().toInt();
    imageSize.width = 640;
    imageSize.height = 480;
    float squareSize = squareEdit->edit->text().toFloat();
    vector<vector<Point2f> > imagePoints;
    vector<Mat> rvecs, tvecs;
    Mat view, viewGray;
    string filename;

    for (int i = 0; i < pic_index; i++)
    {
        filename = dirname.toStdString() + to_string(i) + ".png";
        view = imread(String(filename.c_str()));

        vector<Point2f> pointbuf;
        cvtColor(view, viewGray, COLOR_BGR2GRAY);
        bool found = findChessboardCorners( view, boardSize, pointbuf,
                                            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            cornerSubPix( viewGray, pointbuf, Size(11, 11), Size(-1, -1),
                          TermCriteria( TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1 ));
            drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
            Mat dst;
            cvtColor(view, dst, CV_BGR2RGB);
            QImage *disImage = new QImage((const unsigned char *)(dst.data), dst.cols, dst.rows, QImage::Format_RGB888);
            imgLab->setPixmap(QPixmap::fromImage(disImage->scaled(imgLab->size(), Qt::KeepAspectRatio)));
            imagePoints.push_back(pointbuf);
        }
    }

    if (imagePoints.size() < 5)
    {
        QMessageBox::warning(this, "Error", "Pictures are invalid");
        return;
    }

    distCoeffs = Mat::zeros(8, 1, CV_64F);
    vector<vector<Point3f> > objectPoints(1);

    for ( int i = 0; i < boardSize.height; i++ )
        for ( int j = 0; j < boardSize.width; j++ )
        {
            objectPoints[0].push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
        }

    objectPoints.resize(imagePoints.size(), objectPoints[0]);
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, CALIB_FIX_K4 | CALIB_FIX_K5);
    QString rmserr = "RMS error: " + QString::number(rms);
    statusLab->setText(rmserr);
    cout << "camera_matrix" << cameraMatrix;
    cout << "distortion_coefficients" << distCoeffs;
}

void camera_calibration::closeEvent(QCloseEvent *event)
{
    camThread->exit(0);
}