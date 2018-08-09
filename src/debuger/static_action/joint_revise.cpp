#include "joint_revise.hpp"
#include "robot/humanoid.hpp"

using namespace robot;
using namespace std;

JSlider::JSlider(joint_ptr j, const float &deg): name_(j->name_), id_(j->jid_), range_(100), scale_(10.0)
{
    nameLab = new QLabel(QString::fromStdString(name_));
    nameLab->setFixedWidth(100);
    slider = new QSlider(Qt::Horizontal);
    slider->setMinimumWidth(200);
    slider->setMaximum(range_);
    slider->setMinimum(-range_);
    slider->setValue(static_cast<int>(scale_*deg));
    dataLab = new QLabel(QString::number(deg, 'f', 1));
    dataLab->setFixedWidth(40);
    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addWidget(nameLab);
    mainLayout->addWidget(slider);
    mainLayout->addWidget(dataLab);
    setLayout(mainLayout);
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(procSliderChanged(int)));
}

void JSlider::reset()
{
    slider->setValue(0);
    procSliderChanged(0);
}

void JSlider::procSliderChanged(int v)
{
    float offset = v/scale_;
    dataLab->setText(QString::number(offset, 'f', 1));
    emit valueChanged(id_, offset);
}

joint_revise::joint_revise()
{
    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout = new QVBoxLayout();
    QVBoxLayout *rightLayout = new QVBoxLayout();

    JSlider *slider;
    for(auto jo:ROBOT.offsets_)
    {
        slider = new JSlider(ROBOT.joint_map_[jo.first], jo.second);
        connect(slider, SIGNAL(valueChanged(int, float)), this, SLOT(procValueChanged(int, float)));
        if(jo.first == "jhead2" || jo.first.find("jr") != string::npos)
            rightLayout->addWidget(slider);
        else leftLayout->addWidget(slider);
        j_sliders_[jo.first] = slider;
    }
    btnReset = new QPushButton("Reset");
    btnSave = new QPushButton("Save");
    leftLayout->addWidget(btnReset);
    rightLayout->addWidget(btnSave);
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    setLayout(mainLayout);
    connect(btnReset, SIGNAL(clicked(bool)), this, SLOT(procBtnReset()));
    connect(btnSave, SIGNAL(clicked(bool)), this, SLOT(procBtnSave()));
    setAttribute(Qt::WA_DeleteOnClose);
}

void joint_revise::procBtnReset()
{
    for(auto s:j_sliders_)
    {
        s.second->reset();
        ROBOT.offsets_[s.first] = 0.0;
    }
}

void joint_revise::procBtnSave()
{

}

void joint_revise::procValueChanged(int id, float v)
{
}

joint_revise::~joint_revise()
{
}