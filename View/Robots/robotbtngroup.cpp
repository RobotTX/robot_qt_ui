#include "robotbtngroup.h"
#include <QVBoxLayout>
#include <QAbstractButton>
#include "Controller/mainwindow.h"
#include "Model/Robots/robot.h"
#include "View/Robots/robotview.h"
#include "View/Other/custompushbutton.h"

RobotBtnGroup::RobotBtnGroup(const QVector<QPointer<RobotView>>& vector, MainWindow *mainWindow, QWidget* parent):QWidget(parent){
    btnGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);

    for(int i = 0; i < vector.length(); i++){
        CustomPushButton* robotBtn = new CustomPushButton(vector[i]->getRobot()->getName(), this, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
        connect(robotBtn, SIGNAL(doubleClick(QString)), mainWindow, SLOT(doubleClickOnRobot(QString)));
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
