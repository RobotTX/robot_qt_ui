#include "robotbtngroup.h"
#include "View/robotview.h"
#include "Model/robot.h"
#include <QVBoxLayout>
#include <QPushButton>

RobotBtnGroup::RobotBtnGroup(const QVector<RobotView*>& vector, const bool checkable, QWidget* parent):QWidget(parent){
    btnGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    /// If the buttons are checkable, we can select multiple buttons
    if(checkable){
        btnGroup->setExclusive(false);
    }

    for(int i = 0; i < vector.length(); i++){
        QPushButton* robotBtn = new QPushButton(vector[i]->getRobot()->getName(), this);
        robotBtn->setFlat(true);
        robotBtn->setStyleSheet("text-align:left");
        if(checkable){
            robotBtn->setCheckable(true);
            robotBtn->setChecked(true);
        }
        btnGroup->addButton(robotBtn);
        layout->addWidget(robotBtn);
    }
    hide();
    layout->setAlignment(Qt::AlignTop);
}

RobotBtnGroup::~RobotBtnGroup(){
    delete btnGroup;
    delete layout;
}
