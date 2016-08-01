#include "robotbtngroup.h"
#include "View/robotview.h"
#include "Model/robot.h"
#include <QVBoxLayout>
#include <QPushButton>
#include "doubleclickablebutton.h"
#include <QAbstractButton>

RobotBtnGroup::RobotBtnGroup(const QVector<RobotView*>& vector, const bool checkable, QWidget* parent):QWidget(parent){
    btnGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    for(int i = 0; i < vector.length(); i++){
        DoubleClickableButton* robotBtn = new DoubleClickableButton(vector[i]->getRobot()->getName(),vector[i]->getRobot()->getName(), this);
        connect(robotBtn, SIGNAL(doubleClick(QString)), parent, SLOT(doubleClickOnRobot(QString)));

        robotBtn->setFlat(true);
        robotBtn->setStyleSheet("QPushButton {text-align:left}""QPushButton:hover{text-align:left; background-color: grey; border: 1px;}");
        robotBtn->setCheckable(true);
       // robotBtn->setStyleSheet("QPushButton { border: 1px solid #d3d3d3}""QPushButton:hover{ background-color: grey; border: 1px;}");

        btnGroup->addButton(robotBtn, i);
        layout->addWidget(robotBtn);
    }
    hide();
    layout->setAlignment(Qt::AlignTop);
}
