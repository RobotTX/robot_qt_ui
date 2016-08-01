#include "robotbtngroup.h"
#include "View/robotview.h"
#include "Model/robot.h"
#include <QVBoxLayout>
#include <QPushButton>
#include "doubleclickablebutton.h"
#include "View/colors.h"
#include <QAbstractButton>

RobotBtnGroup::RobotBtnGroup(const QVector<RobotView*>& vector, const bool checkable, QWidget* parent):QWidget(parent){
    btnGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    for(int i = 0; i < vector.length(); i++){
        DoubleClickableButton* robotBtn = new DoubleClickableButton(vector[i]->getRobot()->getName(),vector[i]->getRobot()->getName(), this);
        connect(robotBtn, SIGNAL(doubleClick(QString)), parent, SLOT(doubleClickOnRobot(QString)));

        robotBtn->setFlat(true);
        robotBtn->setStyleSheet("QPushButton {color: "+text_color+";text-align:left;border: 4px; padding: 10px;}QPushButton:hover{background-color: "+button_hover_color+";}QPushButton:checked{background-color: "+button_checked_color+";}");

        robotBtn->setCheckable(true);

        btnGroup->addButton(robotBtn, i);
        layout->addWidget(robotBtn);
    }
    hide();
    layout->setAlignment(Qt::AlignTop);
}
