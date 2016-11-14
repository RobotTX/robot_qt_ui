#include "robotbtngroup.h"
#include "View/robotview.h"
#include "Model/robot.h"
#include <QVBoxLayout>
#include "View/custompushbutton.h"
#include <QAbstractButton>

RobotBtnGroup::RobotBtnGroup(const QVector<QPointer<RobotView>>& vector, QWidget* parent):QWidget(parent){
    btnGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);

    for(int i = 0; i < vector.length(); i++){
        CustomPushButton* robotBtn = new CustomPushButton(vector[i]->getRobot()->getName(), this, false, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
        connect(robotBtn, SIGNAL(doubleClick(QString)), parent, SLOT(doubleClickOnRobot(QString)));
        btnGroup->addButton(robotBtn, i);
        layout->addWidget(robotBtn);
    }
    hide();
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 0, 0);
}

void RobotBtnGroup::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width() - 18;
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
