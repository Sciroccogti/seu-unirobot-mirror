#include <iostream>
#include <fstream>
#include "static_action.hpp"
#include "robot/kinematics.hpp"
#include "math/math.hpp"
#include "joint_revise.hpp"
#include "class_exception.hpp"
#define SCALE_K  0.001f
#define DEG_RANGE 180.0f

using namespace parser;
using namespace std;
using namespace robot;
using namespace robot_math;
using namespace Eigen;

CPosListWidget::CPosListWidget(const int &id, const std::string &p_name, const int &t, const std::string &a_name)
:time_(t), pos_name_(p_name), act_name_(a_name)
{
    m_id = new QLabel(QString::number(id));
    pos_name = new QLineEdit(QString::fromStdString(p_name));
    pos_time = new QLineEdit(QString::number(t));
    pos_time->setFixedWidth(40);
    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addWidget(m_id);
    mainLayout->addWidget(pos_name);
    mainLayout->addWidget(pos_time);
    setLayout(mainLayout);
    connect(pos_name, SIGNAL(editingFinished()), this, SLOT(procNameChange()));
    connect(pos_time, SIGNAL(editingFinished()), this, SLOT(procTimeChange()));
}

void CPosListWidget::procNameChange()
{
    std::string name = pos_name->text().toStdString();
    if(ROBOT.poses_.find(name)==ROBOT.poses_.end())
    {
        robot_pos tempPos;
        tempPos.name = name;
        tempPos.pose_info = ROBOT.poses_[pos_name_].pose_info;
        tempPos.joints_deg = ROBOT.poses_[pos_name_].joints_deg;
        ROBOT.poses_[name] = tempPos;
    }
    robot_act &act = ROBOT.acts_[act_name_];
    for(auto &p:act.poses)
    {
        if(p.pos_name==pos_name_)
        {
            p.pos_name = name;
            pos_name_ = name;
            break;
        }
    }
}

void CPosListWidget::procTimeChange()
{
    robot_act &act = ROBOT.acts_[act_name_];
    for(auto &p:act.poses)
    {
        if(p.pos_name==pos_name_)
        {
            p.act_time = pos_time->text().toInt();
            break;
        }
    }
}

static_action::static_action(const int &id): id_(id)
{
    initRobot();
    //setWindowFlags(Qt::WindowStaysOnTopHint);
    initStatusBar();
    robot_gl_ = new robot_gl();
    QVBoxLayout *leftLayout = new QVBoxLayout;
    leftLayout->addWidget(robot_gl_);

    m_pPoseListWidget = new QListWidget;
    m_pPoseListWidget->setFixedHeight(370);
    mButtonSavePos = new QPushButton(tr("Save Pos"));
    head = new QRadioButton("Head");
    body = new QRadioButton("Body");
    leftArm = new QRadioButton("Left Arm");
    rightArm = new QRadioButton("Right Arm");
    leftFoot = new QRadioButton("Left Foot");
    rightFoot = new QRadioButton("Right Foot");

    m_pJDListWidget1 = new QListWidget;
    m_pJDListWidget2 = new QListWidget;

    motionBtnGroup = new QButtonGroup();
    motionBtnGroup->addButton(head, static_cast<int>(MOTION_HEAD));
    motionBtnGroup->addButton(body, static_cast<int>(MOTION_BODY));
    motionBtnGroup->addButton(leftArm, static_cast<int>(MOTION_LEFT_HAND));
    motionBtnGroup->addButton(rightArm, static_cast<int>(MOTION_RIGHT_HAND));
    motionBtnGroup->addButton(leftFoot, static_cast<int>(MOTION_LEFT_FOOT));
    motionBtnGroup->addButton(rightFoot, static_cast<int>(MOTION_RIGHT_FOOT));
    body->setChecked(true);

    QHBoxLayout *hbLayout = new QHBoxLayout();
    hbLayout->addWidget(head);
    hbLayout->addWidget(body);
    QHBoxLayout *armLayout = new QHBoxLayout();
    armLayout->addWidget(rightArm);
    armLayout->addWidget(leftArm);
    QHBoxLayout *footLayout = new QHBoxLayout();
    footLayout->addWidget(rightFoot);
    footLayout->addWidget(leftFoot);
    QHBoxLayout *jdLayout = new QHBoxLayout();
    jdLayout->addWidget(m_pJDListWidget1);
    jdLayout->addWidget(m_pJDListWidget2);


    QVBoxLayout *midLayout = new QVBoxLayout;
    midLayout->setAlignment(Qt::AlignCenter);
    midLayout->addWidget(m_pPoseListWidget);
    midLayout->addLayout(hbLayout);
    midLayout->addLayout(armLayout);
    midLayout->addLayout(footLayout);
    midLayout->addLayout(jdLayout);
    midLayout->addWidget(mButtonSavePos);

    m_pPosListWidget = new QListWidget();
    m_pPosListWidget->setMinimumWidth(240);
    btnrunPos = new QPushButton("Run Pos");
    btnWalkRemote = new QPushButton("Walk Remote");
    btnJointRevise = new QPushButton("Joint Revise");
    QVBoxLayout *posLayout = new QVBoxLayout;
    posLayout->addWidget(m_pPosListWidget);
    posLayout->addWidget(btnrunPos);

    m_pActListWidget = new QListWidget();
    mButtonInsertPosFront = new QPushButton(tr("Insert Pos Front"));
    mButtonInsertPosBack = new QPushButton(tr("Insert Pos Back"));
    mButtonDeletePos = new QPushButton(tr("Delete Pos"));
    mButtonSaveAction = new QPushButton(tr("Save Actions"));
    mButtonAddAction = new QPushButton(tr("Add Action"));
    mButtonDeleteAction = new QPushButton(tr("Delete Action"));

    QVBoxLayout *bactLayout = new QVBoxLayout;
    bactLayout->addWidget(mButtonInsertPosFront);
    bactLayout->addWidget(mButtonInsertPosBack);
    bactLayout->addWidget(mButtonDeletePos);
    bactLayout->addWidget(mButtonSaveAction);
    bactLayout->addWidget(mButtonAddAction);
    bactLayout->addWidget(mButtonDeleteAction);
    bactLayout->addWidget(m_pActListWidget);
    bactLayout->addWidget(btnWalkRemote);
    bactLayout->addWidget(btnJointRevise);

    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(midLayout);
    mainLayout->addLayout(posLayout);
    mainLayout->addLayout(bactLayout);
    

    QWidget *mainWidget  = new QWidget();
    mainWidget->setLayout(mainLayout);
    this->setCentralWidget(mainWidget);

    initSlider();
    initActs();
    initPoseMap();
    initJDInfo();

    connect(m_pActListWidget, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(procActSelect(QListWidgetItem*)));
    connect(m_pPosListWidget, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(procPosSelect(QListWidgetItem*)));
    connect(mButtonAddAction, SIGNAL(clicked(bool)), this, SLOT(procButtonAddAction()));
    connect(mButtonDeleteAction, SIGNAL(clicked(bool)), this, SLOT(procButtonDeleteAction()));
    connect(mButtonSaveAction, SIGNAL(clicked(bool)), this, SLOT(procButtonSaveAction()));
    connect(mButtonInsertPosFront, SIGNAL(clicked(bool)), this, SLOT(procButtonInsertPosFront()));
    connect(mButtonInsertPosBack, SIGNAL(clicked(bool)), this, SLOT(procButtonInsertPosBack()));
    connect(mButtonDeletePos, SIGNAL(clicked(bool)), this, SLOT(procButtonDeletePos()));
    connect(mButtonSavePos, SIGNAL(clicked(bool)), this, SLOT(procButtonSavePos()));
    connect(btnrunPos, SIGNAL(clicked(bool)), this, SLOT(procButtonRunPos()));
    connect(btnWalkRemote, SIGNAL(clicked(bool)), this, SLOT(procButtonWalkRemote()));
    connect(btnJointRevise,SIGNAL(clicked(bool)),this,SLOT(procButtonJointRevise()));
    connect(motionBtnGroup, SIGNAL(buttonClicked(int)), this, SLOT(updateSlider(int)));
}

void static_action::initRobot()
{
    if(!config_parser::parse("data/actuator.conf", config_tree_)) 
        throw class_exception<static_action>("get config failed");
    player_ = "players." + to_string(id_);
    ROBOT.init(config_tree_.get<std::string>(player_+".robot_file"), 
                config_tree_.get<std::string>(player_+".action_file"), 
                config_tree_.get<std::string>(player_+".offset_file"));
}

void static_action::initActs()
{
    m_pPosListWidget->clear();
    m_pActListWidget->clear();
    for(auto act:ROBOT.acts_)
        m_pActListWidget->addItem(QString::fromStdString(act.second.name));
    m_pPoseListWidget->setEnabled(false);
}

void static_action::initPoseMap()
{
    motion_ = MOTION_BODY;
    robot_pose temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for(int i=1;i<=6;i++)
        pose_map_[static_cast<robot_motion >(i)] = temp;
    for(auto j:ROBOT.joint_map_)
        joint_degs_[j.first] = j.second->get_deg();
}

void static_action::initStatusBar()
{
    QStatusBar *bar = statusBar();
    motionlab = new QLabel;
    valuelab = new QLabel;
    curractlab = new QLabel;
    currposlab = new QLabel;
    netstatuslab = new QLabel;
    motionlab->setMinimumWidth(60);
    valuelab->setMinimumWidth(60);
    curractlab->setMinimumWidth(60);
    currposlab->setMinimumWidth(60);
    netstatuslab->setMinimumWidth(60);
    motionlab->setFrameShape(QFrame::WinPanel);
    motionlab->setFrameShadow(QFrame::Sunken);
    valuelab->setFrameShape(QFrame::WinPanel);
    valuelab->setFrameShadow(QFrame::Sunken);
    curractlab->setFrameShape(QFrame::WinPanel);
    curractlab->setFrameShadow(QFrame::Sunken);
    currposlab->setFrameShape(QFrame::WinPanel);
    currposlab->setFrameShadow(QFrame::Sunken);
    netstatuslab->setFrameShape(QFrame::WinPanel);
    netstatuslab->setFrameShadow(QFrame::Sunken);
    bar->addWidget(motionlab);
    bar->addWidget(valuelab);
    bar->addWidget(curractlab);
    bar->addWidget(currposlab);
    bar->addWidget(netstatuslab);
}

void static_action::initSlider()
{
    mKsliders.clear();
    CKSlider *slider;

    slider = new CKSlider("X");
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(procX(int)));
    mKsliders.push_back(slider);
    slider = new CKSlider("Y");
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(procY(int)));
    mKsliders.push_back(slider);
    slider = new CKSlider("Z");
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(procZ(int)));
    mKsliders.push_back(slider);
    slider = new CKSlider("Roll");
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(procRoll(int)));
    mKsliders.push_back(slider);
    slider = new CKSlider("Pitch");
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(procPitch(int)));
    mKsliders.push_back(slider);
    slider = new CKSlider("Yaw");
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(procYaw(int)));
    mKsliders.push_back(slider);

    QListWidgetItem *plistItem;
    for(auto s:mKsliders)
    {
        plistItem = new QListWidgetItem;
        s->show();
        m_pPoseListWidget->addItem(plistItem);
        m_pPoseListWidget->setItemWidget(plistItem, s);
        plistItem->setSizeHint(QSize(s->rect().width(), s->rect().height()));
    }
}

void static_action::initJDInfo()
{
    mJDInfos.clear();
    CJointDegWidget *pJDWidget;

    QListWidgetItem *pListItem;;
    m_pJDListWidget1->clear();
    m_pJDListWidget2->clear();
    for(auto jd: joint_degs_)
    {
        pListItem = new QListWidgetItem;
        pJDWidget = new CJointDegWidget(jd.first, jd.second);
        pJDWidget->show();
        mJDInfos[jd.first] = pJDWidget;
        if(jd.first == "jhead2" || jd.first.find("jr") != string::npos)
        {
            m_pJDListWidget1->addItem(pListItem);
            m_pJDListWidget1->setItemWidget(pListItem, pJDWidget);
        }
        else
        {
            m_pJDListWidget2->addItem(pListItem);
            m_pJDListWidget2->setItemWidget(pListItem, pJDWidget);
        }
        pListItem->setSizeHint(QSize(pJDWidget->rect().width(), pJDWidget->rect().height()));
    }
}

void static_action::updateSlider(int id)
{
    motion_ = static_cast<robot_motion >(id);
    motionlab->setText(QString::fromStdString(get_name_by_motion(motion_)));

    std::vector<CKSlider*>::iterator iter = mKsliders.begin();
    (*iter)->slider->setValue(pose_map_[motion_].x / SCALE_K);
    (*iter)->name->setText("X");
    if(motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND)
        (*iter)->name->setText("Jshoulder1");
    else if(motion_ == MOTION_HEAD)
        (*iter)->name->setText("Head Pitch");

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].y / SCALE_K);
    (*iter)->name->setText("Y");
    (*iter)->slider->setEnabled(true);
    if(motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND)
    {
        (*iter)->name->setText("Jshoulder2");
        (*iter)->slider->setEnabled(false);
    }
    else if(motion_ == MOTION_HEAD)
        (*iter)->name->setText("Head Yaw");

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].z / SCALE_K);
    (*iter)->name->setText("Z");
    if(motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND)
        (*iter)->name->setText("Jelbow");
    (*iter)->slider->setEnabled(true);
    if(motion_ == MOTION_HEAD)
        (*iter)->slider->setEnabled(false);

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].roll);
    (*iter)->slider->setEnabled(true);
    if(motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND || motion_ == MOTION_HEAD)
        (*iter)->slider->setEnabled(false);

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].pitch);
    (*iter)->slider->setEnabled(true);
    if(motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND || motion_ == MOTION_HEAD)
        (*iter)->slider->setEnabled(false);

    iter++;
    (*iter)->slider->setValue(pose_map_[motion_].yaw);
    (*iter)->slider->setEnabled(true);
    if(motion_ == MOTION_LEFT_HAND || motion_ == MOTION_RIGHT_HAND || motion_ == MOTION_HEAD)
        (*iter)->slider->setEnabled(false);
}


float static_action::get_deg_from_pose(const float &ps)
{
    int value = (int)(ps/SCALE_K);
    return DEG_RANGE/SLIDER_RANGE*value;
}

bool static_action::calculate()
{
    robot_pose pose = pose_map_[MOTION_LEFT_HAND];
    joint_degs_["jhead2"] = get_deg_from_pose(pose_map_[MOTION_HEAD].x);
    joint_degs_["jhead1"] = get_deg_from_pose(pose_map_[MOTION_HEAD].y);
    joint_degs_["jrshoulder1"] = get_deg_from_pose(pose_map_[MOTION_RIGHT_HAND].x);
    joint_degs_["jrelbow"] = get_deg_from_pose(pose_map_[MOTION_RIGHT_HAND].z);
    joint_degs_["jlshoulder1"] = get_deg_from_pose(pose_map_[MOTION_LEFT_HAND].x);
    joint_degs_["jlelbow"] = -get_deg_from_pose(pose_map_[MOTION_LEFT_HAND].z);

    transform_matrix body_mat, leftfoot_mat, rightfoot_mat;
    double cx,cy,cz,sx,sy,sz;
    Matrix3d R;
    body_mat.set_p(Vector3d(pose_map_[MOTION_BODY].x, pose_map_[MOTION_BODY].y, pose_map_[MOTION_BODY].z+RK.A+RK.B+RK.E));
    cx = cos(deg2rad(pose_map_[MOTION_BODY].roll));
    cy = cos(deg2rad(pose_map_[MOTION_BODY].pitch));
    cz = cos(deg2rad(pose_map_[MOTION_BODY].yaw));
    sx = sin(deg2rad(pose_map_[MOTION_BODY].roll));
    sy = sin(deg2rad(pose_map_[MOTION_BODY].pitch));
    sz = sin(deg2rad(pose_map_[MOTION_BODY].yaw));
    R<<cy * cz - sx * sy * sz, -cx * sz, sy * cz + cy * sx * sz,
       cy * sz + sx * sy * cz, cx * cz, sy * sz - cy * sx * cz,
       -cx * sy, sx, cx * cy;
    body_mat.set_R(R);

    leftfoot_mat.set_p(Vector3d(pose_map_[MOTION_LEFT_FOOT].x, pose_map_[MOTION_LEFT_FOOT].y+RK.D, pose_map_[MOTION_LEFT_FOOT].z));
    cx = cos(deg2rad(pose_map_[MOTION_LEFT_FOOT].roll));
    cy = cos(deg2rad(pose_map_[MOTION_LEFT_FOOT].pitch));
    cz = cos(deg2rad(pose_map_[MOTION_LEFT_FOOT].yaw));
    sx = sin(deg2rad(pose_map_[MOTION_LEFT_FOOT].roll));
    sy = sin(deg2rad(pose_map_[MOTION_LEFT_FOOT].pitch));
    sz = sin(deg2rad(pose_map_[MOTION_LEFT_FOOT].yaw));
    R<<cy * cz - sx * sy * sz, -cx * sz, sy * cz + cy * sx * sz,
       cy * sz + sx * sy * cz, cx * cz, sy * sz - cy * sx * cz,
       -cx * sy, sx, cx * cy;
    leftfoot_mat.set_R(R);

    rightfoot_mat.set_p(Vector3d(pose_map_[MOTION_RIGHT_FOOT].x, pose_map_[MOTION_RIGHT_FOOT].y-RK.D, pose_map_[MOTION_RIGHT_FOOT].z));
    cx = cos(deg2rad(pose_map_[MOTION_RIGHT_FOOT].roll));
    cy = cos(deg2rad(pose_map_[MOTION_RIGHT_FOOT].pitch));
    cz = cos(deg2rad(pose_map_[MOTION_RIGHT_FOOT].yaw));
    sx = sin(deg2rad(pose_map_[MOTION_RIGHT_FOOT].roll));
    sy = sin(deg2rad(pose_map_[MOTION_RIGHT_FOOT].pitch));
    sz = sin(deg2rad(pose_map_[MOTION_RIGHT_FOOT].yaw));
    R<<cy * cz - sx * sy * sz, -cx * sz, sy * cz + cy * sx * sz,
       cy * sz + sx * sy * cz, cx * cz, sy * sz - cy * sx * cz,
       -cx * sy, sx, cx * cy;
    rightfoot_mat.set_R(R);

    vector<double> degs;
    if(RK.leg_inverse_kinematics(body_mat, leftfoot_mat, degs, 1.0))
    {
        joint_degs_["jlhip3"] = rad2deg(degs[0]);
        joint_degs_["jlhip2"] = rad2deg(degs[1]);
        joint_degs_["jlhip1"] = rad2deg(degs[2]);
        joint_degs_["jlknee"] = rad2deg(degs[3]);
        joint_degs_["jlankle2"] = rad2deg(degs[4]);
        joint_degs_["jlankle1"] = rad2deg(degs[5]);
    }else return false;

    if(RK.leg_inverse_kinematics(body_mat, rightfoot_mat, degs, -1.0))
    {
        joint_degs_["jrhip3"] = rad2deg(degs[0]);
        joint_degs_["jrhip2"] = rad2deg(degs[1]);
        joint_degs_["jrhip1"] = rad2deg(degs[2]);
        joint_degs_["jrknee"] = rad2deg(degs[3]);
        joint_degs_["jrankle2"] = rad2deg(degs[4]);
        joint_degs_["jrankle1"] = rad2deg(degs[5]);
    }else return false;

    return true;
}

void static_action::procX(int value)
{
    float old_value = pose_map_[motion_].x;
    float v = ((float) value) * SCALE_K;
    valuelab->setText(QString::number(v, 'f'));
    pose_map_[motion_].x = v;
    if(leftArm->isChecked()) pose_map_[MOTION_LEFT_HAND].x = v;
    if(rightArm->isChecked()) pose_map_[MOTION_RIGHT_HAND].x = v;
    if(leftFoot->isChecked()) pose_map_[MOTION_LEFT_FOOT].x = v;
    if(rightFoot->isChecked()) pose_map_[MOTION_RIGHT_FOOT].x = v;
    if(calculate())
    {
        robot_gl_->turn_joint(joint_degs_);
        updateJDInfo();
    }
    else pose_map_[motion_].x = old_value;
}

void static_action::procY(int value)
{
    float old_value = pose_map_[motion_].y;
    float v = ((float) value) * SCALE_K;
    valuelab->setText(QString::number(v, 'f'));
    pose_map_[motion_].y = v;
    if(leftFoot->isChecked()) pose_map_[MOTION_LEFT_FOOT].y = v;
    if(rightFoot->isChecked()) pose_map_[MOTION_RIGHT_FOOT].y = v;
    if(calculate())
    {
        robot_gl_->turn_joint(joint_degs_);
        updateJDInfo();
    }
    else pose_map_[motion_].y = old_value;
}

void static_action::procZ(int value)
{
    float old_value = pose_map_[motion_].z;
    float v = ((float) value) * SCALE_K;
    valuelab->setText(QString::number(v, 'f'));
    pose_map_[motion_].z = v;
    if(leftArm->isChecked()) pose_map_[MOTION_LEFT_HAND].z = v;
    if(rightArm->isChecked()) pose_map_[MOTION_RIGHT_HAND].z = v;
    if(leftFoot->isChecked()) pose_map_[MOTION_LEFT_FOOT].z = v;
    if(rightFoot->isChecked()) pose_map_[MOTION_RIGHT_FOOT].z = v;
    if(calculate())
    {
        robot_gl_->turn_joint(joint_degs_);
        updateJDInfo();
    }
    else pose_map_[motion_].z = old_value;
}

void static_action::procRoll(int value)
{
    float old_value = pose_map_[motion_].roll;
    valuelab->setText(QString::number(((float) value), 'f'));
    pose_map_[motion_].roll = ((float) value);
    if(leftFoot->isChecked()) pose_map_[MOTION_LEFT_FOOT].roll = ((float) value);
    if(rightFoot->isChecked()) pose_map_[MOTION_RIGHT_FOOT].roll = ((float) value);
    if(calculate())
    {
        robot_gl_->turn_joint(joint_degs_);
        updateJDInfo();
    }
    else pose_map_[motion_].roll = old_value;
}

void static_action::procPitch(int value)
{
    float old_value = pose_map_[motion_].pitch;
    valuelab->setText(QString::number(((float) value), 'f'));
    pose_map_[motion_].pitch = ((float) value);
    if(leftFoot->isChecked()) pose_map_[MOTION_LEFT_FOOT].pitch = ((float) value);
    if(rightFoot->isChecked()) pose_map_[MOTION_RIGHT_FOOT].pitch = ((float) value);
    if(calculate())
    {
        robot_gl_->turn_joint(joint_degs_);
        updateJDInfo();
    }
    else pose_map_[motion_].pitch = old_value;
}

void static_action::procYaw(int value)
{
    float old_value = pose_map_[motion_].yaw;
    valuelab->setText(QString::number(((float) value), 'f'));
    pose_map_[motion_].yaw = ((float) value);
    if(leftFoot->isChecked()) pose_map_[MOTION_LEFT_FOOT].yaw = ((float) value);
    if(rightFoot->isChecked()) pose_map_[MOTION_RIGHT_FOOT].yaw = ((float) value);
    if(calculate())
    {
        robot_gl_->turn_joint(joint_degs_);
        updateJDInfo();
    }
    else pose_map_[motion_].yaw = old_value;
}

void static_action::updateJDInfo()
{
    for(auto jd: joint_degs_)
        mJDInfos[jd.first]->deg->setText(QString::number(jd.second,'f', 2));
    update();
}

void static_action::updatePosList(string act_name)
{
    robot_act act = ROBOT.acts_[act_name];
    QListWidgetItem *pListItem;
    CPosListWidget *pPosWidget;
    int id = 0;
    m_pPosListWidget->clear();
    for(auto pos:act.poses)
    {
        id++;
        pListItem = new QListWidgetItem;
        pPosWidget = new CPosListWidget(id, pos.pos_name, pos.act_time, act.name);
        pPosWidget->show();
        m_pPosListWidget->addItem(pListItem);
        m_pPosListWidget->setItemWidget(pListItem, pPosWidget);
        pListItem->setSizeHint(QSize(pPosWidget->rect().width(), pPosWidget->rect().height()));
    }
}

void static_action::procActSelect(QListWidgetItem* item)
{
    curractlab->setText(item->text());
    currposlab->setText("");
    string name = m_pActListWidget->currentItem()->text().toStdString();
    updatePosList(name);
    m_pPoseListWidget->setEnabled(false);
}

void static_action::procPosSelect(QListWidgetItem* item)
{
    CPosListWidget *pPosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(item);
    string pos_name = pPosWidget->pos_name_;
    currposlab->setText(QString::fromStdString(pos_name));
    valuelab->setText("");
    pose_map_ = ROBOT.poses_[pos_name].pose_info;
    joint_degs_ = ROBOT.poses_[pos_name].joints_deg;
    for(auto jd: joint_degs_)
        ROBOT.joint_map_[jd.first]->set_deg(jd.second);
    updateJDInfo();
    updateSlider(static_cast<int>(motion_));
    robot_gl_->turn_joint(joint_degs_);
    m_pPoseListWidget->setEnabled(true);
}

void static_action::procButtonInsertPosFront()
{
    if(m_pActListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No act select!");
        return;
    }
    string act_name = m_pActListWidget->currentItem()->text().toStdString();

    if(m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }

    bool ok;
    string new_pos_name = QInputDialog::getText(this,tr("act name"),tr("input act name:"),
                                        QLineEdit::Normal, nullptr, &ok).toStdString();
    if(!ok || new_pos_name.empty()) return;
    bool exist = false;
    if(ROBOT.poses_.find(new_pos_name) != ROBOT.poses_.end())
    {
        exist = true;
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos: "+QString::fromStdString(new_pos_name)+"  exists, use it?",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
        if(reply != QMessageBox::StandardButton::Yes) return;
    }
    robot_pos pos;
    robot_one_pos one_pos;
    one_pos.act_time = 100;
    one_pos.pos_name = new_pos_name;

    pos.name = new_pos_name;
    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    string pos_name = pCur_PosWidget->pos_name_;
    pos.joints_deg = ROBOT.poses_[pos_name].joints_deg;
    pos.pose_info = ROBOT.poses_[pos_name].pose_info;
    int id = pCur_PosWidget->m_id->text().toInt();
    ROBOT.acts_[act_name].poses.insert(ROBOT.acts_[act_name].poses.begin()+id-1, one_pos);
    if(!exist) ROBOT.poses_[new_pos_name] = pos;
    updatePosList(act_name);
}

void static_action::procButtonInsertPosBack()
{
    if(m_pActListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No act select!");
        return;
    }
    string act_name = m_pActListWidget->currentItem()->text().toStdString();

    bool ok;
    string new_pos_name = QInputDialog::getText(this,tr("pos name"),tr("input pos name:"), QLineEdit::Normal, nullptr, &ok).toStdString();
    if(!ok || new_pos_name.empty()) return;
    bool exist = false;
    if(ROBOT.poses_.find(new_pos_name) != ROBOT.poses_.end())
    {
        exist = true;
        QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "pos: "+QString::fromStdString(new_pos_name)+"  exists, use it?",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
        if(reply != QMessageBox::StandardButton::Yes) return;
    }
    robot_pos pos;
    robot_one_pos one_pos;
    one_pos.act_time = 100;
    one_pos.pos_name = new_pos_name;

    pos.name = new_pos_name;
    if(m_pPosListWidget->currentItem() == nullptr)
    {
        robot_pose pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        for(int i=1;i<=6;i++)
            pos.pose_info[static_cast<robot_motion >(i)] = pose;
        for(auto j: ROBOT.joint_map_)
            pos.joints_deg[j.first] = 0.0;
        ROBOT.acts_[act_name].poses.push_back(one_pos);
    }
    else
    {
        CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
        string pos_name = pCur_PosWidget->pos_name_;
        pos.joints_deg = ROBOT.poses_[pos_name].joints_deg;
        pos.pose_info = ROBOT.poses_[pos_name].pose_info;
        int id = pCur_PosWidget->m_id->text().toInt();
        ROBOT.acts_[act_name].poses.insert(ROBOT.acts_[act_name].poses.begin()+id, one_pos);
    }
    if(!exist) ROBOT.poses_[new_pos_name] = pos;
    updatePosList(act_name);
}

void static_action::procButtonDeletePos()
{
    if(m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }
    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    int id = pCur_PosWidget->m_id->text().toInt();
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    ROBOT.acts_[act_name].poses.erase(ROBOT.acts_[act_name].poses.begin()+id-1);
    removeUnusedPos();
    updatePosList(act_name);
}

void static_action::procButtonSavePos()
{
    if(m_pPosListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No pos select!");
        return;
    }
    CPosListWidget *pCur_PosWidget = (CPosListWidget *) m_pPosListWidget->itemWidget(m_pPosListWidget->currentItem());
    string pos_name = pCur_PosWidget->pos_name_;
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "current pos is: "+QString::fromStdString(pos_name) + ", save?",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(reply != QMessageBox::StandardButton::Yes) return;

    ROBOT.poses_[pos_name].joints_deg = joint_degs_;
    ROBOT.poses_[pos_name].pose_info = pose_map_;
}

void static_action::procButtonDeleteAction()
{
    if(m_pActListWidget->currentItem() == nullptr)
    {
        QMessageBox::warning(this, "Error", "No act select!");
        return;
    }
    string act_name = m_pActListWidget->currentItem()->text().toStdString();
    auto iter = ROBOT.acts_.begin();
    while (iter!=ROBOT.acts_.end())
    {
        if(iter->first == act_name)
        {
            ROBOT.acts_.erase(iter);
            break;
        }
        iter++;
    }
    removeUnusedPos();
    initActs();
}

void static_action::procButtonSaveAction()
{
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Warning", "all action data will be written into file, save?",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(reply != QMessageBox::StandardButton::Yes) return;
    action_parser::save(config_tree_.get<std::string>(player_+".action_file"), ROBOT.acts_, ROBOT.poses_);
}

void static_action::procButtonAddAction()
{
    bool ok;
    string name = QInputDialog::getText(this,tr("act name"),tr("input act name:"),QLineEdit::Normal, nullptr, &ok).toStdString();
    if(name.empty()) return;
    if(ok)
    {
        if(ROBOT.acts_.find(name) != ROBOT.acts_.end())
        {
            QMessageBox::warning(this, "Waring", "the name had been used", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
            return;
        }
        robot_act act;
        act.name = name;
        ROBOT.acts_[name] = act;
        initActs();
    }
}

void static_action::removeUnusedPos()
{
    bool fd;
    auto p_iter = ROBOT.poses_.begin();
    while(p_iter!=ROBOT.poses_.end())
    {
        fd = false;
        for(auto act:ROBOT.acts_)
        {
            for(auto p:act.second.poses)
            {
                if(p.pos_name == p_iter->first)
                {
                    fd = true;
                    break;
                }
            }
            if(fd) break;
        }
        if(!fd) p_iter = ROBOT.poses_.erase(p_iter);
        else p_iter++;
    }
}

void static_action::procButtonRunPos()
{
   
}

void static_action::procButtonWalkRemote()
{

}

void static_action::procButtonJointRevise()
{
    joint_revise *jr = new joint_revise();
    jr->show();
}

static_action::~static_action()
{
}