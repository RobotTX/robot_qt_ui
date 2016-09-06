#include "pathwidget.h"
#include "View/robotview.h"
#include "Model/pathpoint.h"
#include "Model/robot.h"
#include "Model/points.h"
#include <QVBoxLayout>
#include <QLabel>

PathWidget::PathWidget(QWidget* _parent):QWidget(_parent){
    parent = _parent;

    layout = new QVBoxLayout(this);

    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(0, 0, 0, 0);
}

PathWidget::~PathWidget(){
    delete layout;
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

void PathWidget::setPath(const QVector<QSharedPointer<PathPoint> > &path){

    clearLayout(layout);
    for(int i = 0; i < path.size(); i++){
        /// Index & name of the point

        QLabel* nameLabel = new QLabel(this);
        if(path.at(i)->getPoint().getName().contains(PATH_POINT_NAME))
            nameLabel->setText(QString::number(i+1)+". "+QString::number(path.at(i)->getPoint().getPosition().getX(),'f', 1) + "; " + QString::number(path.at(i)->getPoint().getPosition().getY(),'f', 1));
        else
            nameLabel->setText(QString::number(i+1) + ". " + path.at(i)->getPoint().getName());

        nameLabel->setWordWrap(true);
        nameLabel->setMinimumWidth(1);
        layout->addWidget(nameLabel);

        /// Action to do (wait for human or a certain amount of time)
        QLabel* actionLabel = new QLabel(this);
        if(path.at(i)->getAction() == PathPoint::HUMAN_ACTION){
            actionLabel->setText("Wait for Human Action");
        } else {
            actionLabel->setText("Wait for " + QString::number(path.at(i)->getWaitTime()) + QString(" s"));
        }

        actionLabel->setWordWrap(true);
        actionLabel->setMinimumWidth(1);
        layout->addWidget(actionLabel);

        if(i == path.size()-1){
            actionLabel->hide();
        }
    }
}
