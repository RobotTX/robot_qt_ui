#include "robotbtngroup.h"
#include <QVBoxLayout>
#include <QAbstractButton>
#include <QPushButton>
#include <QDebug>
#include "Model/Robots/robot.h"
#include "Model/Robots/robots.h"
#include "View/Robots/robotview.h"
#include "View/Other/custompushbutton.h"

RobotBtnGroup::RobotBtnGroup(QWidget* parent):QWidget(parent){
    btnGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);

    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 0, 0);
}


void RobotBtnGroup::updateRobots(QSharedPointer<Robots> robots){

    /// Remove buttons
    QList<QAbstractButton*> listBtn = btnGroup->buttons();
    for(int i = 0; i < listBtn.size(); i++){
        btnGroup->removeButton(listBtn.at(i));
        layout->removeWidget(listBtn.at(i));
        delete listBtn.at(i);
    }

    /// Add buttons
    for(int j = 0; j < robots->getRobotsVector().size(); j++){
        CustomPushButton* robotBtn = new CustomPushButton(robots->getRobotsVector().at(j)->getRobot()->getName(), this, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
        connect(robotBtn, SIGNAL(doubleClick(QString)), this, SLOT(doubleClickOnRobotSlot(QString)));
        btnGroup->addButton(robotBtn, j);
        layout->addWidget(robotBtn);
    }
}

void RobotBtnGroup::resizeEvent(QResizeEvent *event){
    int maxWidth = static_cast<QWidget*>(parent())->width() - 18;
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}

void RobotBtnGroup::uncheck(){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    btnGroup->setExclusive(false);
    if(btnGroup->checkedButton())
        btnGroup->checkedButton()->setChecked(false);
    btnGroup->setExclusive(true);
}

void RobotBtnGroup::doubleClickOnRobotSlot(QString robotName){
    emit doubleClickOnRobot(robotName);
}
