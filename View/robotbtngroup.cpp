#include "robotbtngroup.h"
#include "View/robotview.h"
#include "Model/robot.h"
#include <QVBoxLayout>
#include <QPushButton>
#include "doubleclickablebutton.h"

RobotBtnGroup::RobotBtnGroup(const QVector<RobotView*>& vector, const bool checkable, QWidget* parent):QWidget(parent){
    btnGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);
/*
    /// If the buttons are checkable, we can select multiple buttons
    if(checkable){
        btnGroup->setExclusive(false);
    }
*/
    for(int i = 0; i < vector.length(); i++){
     //   QPushButton* robotBtn = new QPushButton(vector[i]->getRobot()->getName(), this);
       DoubleClickableButton* robotBtn = new DoubleClickableButton(i,vector[i]->getRobot()->getName(), this);
       connect(robotBtn, SIGNAL(doubleClick(int)), parent, SLOT(doubleClickOnRobot(int)));

        robotBtn->setFlat(true);
        robotBtn->setStyleSheet("text-align:left");
        robotBtn->setCheckable(true);

        btnGroup->addButton(robotBtn, i);
        layout->addWidget(robotBtn);
    }
    hide();
    layout->setAlignment(Qt::AlignTop);
}
