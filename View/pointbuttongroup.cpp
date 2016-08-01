#include "pointbuttongroup.h"
#include "Model/points.h"
#include "Model/point.h"
#include <QButtonGroup>
#include <QVBoxLayout>
#include "View/doubleclickablebutton.h"
#include <QMouseEvent>
#include <QDebug>
#include <QLabel>
#include "View/pointview.h"

PointButtonGroup::PointButtonGroup(std::shared_ptr<Points> const points, const QString _groupIndex
                                   , QWidget* parent): QWidget(parent){
    layout = new QVBoxLayout(this);

    groupIndex = _groupIndex;
    buttonGroup = new QButtonGroup(this);
    layout->setAlignment(Qt::AlignTop);

    createButtons(points);
}

void PointButtonGroup::setGroup(std::shared_ptr<Points> const points, const QString _groupIndex){
    qDebug() << "PointButtonGroup::setGroup called";
    deleteButtons();
    groupIndex = _groupIndex;
    createButtons(points);
    emit updateConnectionsRequest();
}

void PointButtonGroup::createButtons(std::shared_ptr<Points> const points){
    QMapIterator<QString, std::shared_ptr<QVector<std::shared_ptr<PointView>>>> i(*(points->getGroups()));
    while (i.hasNext()) {
        i.next();
        if(i.value()){
            for(int j = 0; j < i.value()->size(); j++){
                std::shared_ptr<Point> currentPoint = i.value()->at(j)->getPoint();

                DoubleClickableButton* pointButton = new DoubleClickableButton(currentPoint->getName(), currentPoint->getName()
                                                           + " (" + QString::number(currentPoint->getPosition().getX())
                                                           + ", " + QString::number(currentPoint->getPosition().getY()) + ")", this);
                pointButton->setAutoDefault(true);
                pointButton->setFlat(true);
                pointButton->setStyleSheet("text-align:left");
                buttonGroup->addButton(pointButton);
                layout->addWidget(pointButton);
                if(i.value()->at(j)->isVisible())
                    pointButton->setIcon(QIcon(":/icons/eye.png"));
                else
                    pointButton->setIcon(QIcon(":/icons/space_point.png"));
                BUTTON_SIZE = parentWidget()->size()/2 ;
                pointButton->setIconSize(BUTTON_SIZE);
            }
        }
    }
}

void PointButtonGroup::deleteButtons(void){
    qDebug() << "PointButtonGroup::deleteButtons called";
    while(QLayoutItem* item = layout->takeAt(0)){
        if(QWidget* button = item->widget())
            delete button;
    }
}

void PointButtonGroup::setCheckable(const bool checkable){
    qDebug() << "PointButtonGroup::setCheckable called";
    foreach(QAbstractButton* button, buttonGroup->buttons())
        button->setCheckable(checkable);
}

void PointButtonGroup::uncheck(void){
    qDebug() << "PointButtonGroup::uncheck called";
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}
