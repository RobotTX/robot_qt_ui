#include "pathwidget.h"
#include "View/robotview.h"
#include "Model/pathpoint.h"
#include "Model/robot.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

PathWidget::PathWidget(QWidget* _parent):QWidget(_parent){
    parent = _parent;

    layout = new QVBoxLayout(this);

    layout->setAlignment(Qt::AlignTop);
}

void PathWidget::setSelectedRobot(RobotView *const robotView){

    /// Get the path of the robot
    std::shared_ptr<QVector<std::shared_ptr<PathPoint>>> path = robotView->getRobot()->getPath();

    clearLayout(layout);
    for(size_t i = 0; i < path->size(); i++){
        /// Index & name of the point
        QLabel* nameLabel = new QLabel(QString::number(i+1) + " : " + path->at(i)->getPoint().getName(), this);
        nameLabel->setWordWrap(true);
        nameLabel->setMinimumWidth(1);
        layout->addWidget(nameLabel);


        /// Action to do (wait for human or a ertain amount of time)
        QLabel* actionLabel = new QLabel(this);
        if(path->at(i)->getAction() == PathPoint::HUMAN_ACTION){
            actionLabel->setText("Wait for Human Action");
        } else {
            actionLabel->setText("Wait for " + QString::number(path->at(i)->getWaitTime()) + QString(" s"));
        }

        actionLabel->setWordWrap(true);
        actionLabel->setMinimumWidth(1);
        layout->addWidget(actionLabel);

        if(i == path->size()-1){
            actionLabel->hide();
        }
    }
}

void PathWidget::clearLayout(QLayout* _layout){
    QLayoutItem *item = NULL;
    /// We need to delete every item recursively or the item would still be there
    /// but without a layout
    while((item = _layout->takeAt(0))) {
        if (item->layout()) {
            clearLayout(item->layout());
            delete item->layout();
        } else {
            if (item->widget()) {
                delete item->widget();
            }
            _layout->removeItem(item);
            delete item;
        }
    }
}
