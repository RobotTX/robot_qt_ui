#include "robotbtngroup.h"
#include "View/robotview.h"
#include "Model/robot.h"
#include <QVBoxLayout>
#include "View/custompushbutton.h"
#include <QAbstractButton>

RobotBtnGroup::RobotBtnGroup(const QVector<RobotView*>& vector, QWidget* parent):QWidget(parent){
    btnGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);


    for(int i = 0; i < vector.length(); i++){
        CustomPushButton* robotBtn = new CustomPushButton(vector[i]->getRobot()->getName(), this, CustomPushButton::ButtonType::LEFT_MENU, "left", true);
        connect(robotBtn, SIGNAL(doubleClick(QString)), parent, SLOT(doubleClickOnRobot(QString)));

        btnGroup->addButton(robotBtn, i);
        layout->addWidget(robotBtn);
    }
    hide();
    layout->setAlignment(Qt::AlignTop);
}

void RobotBtnGroup::resizeEvent(QResizeEvent *event){
    QWidget* widget = static_cast<QWidget*>(parent());
    int maxWidth = widget->width()-widget->contentsMargins().right()-widget->contentsMargins().left();

    /*qDebug() << "RobotBtnGroup::resizeEvent" << width() << static_cast<QWidget*>(parent())->width()
                 << static_cast<QWidget*>(parent()->parent())->width()
                     << static_cast<QWidget*>(parent()->parent()->parent())->width()
                     << maxWidth;*/
    setMaximumWidth(maxWidth);
    QWidget::resizeEvent(event);
}
