#include "pointbuttongroup.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"
#include <QButtonGroup>
#include <QVBoxLayout>
#include "View/doubleclickablebutton.h"
#include <QMouseEvent>
#include <QDebug>

PointButtonGroup::PointButtonGroup(const Points &_points, const unsigned int groupIndex
                                   , QWidget* parent): QWidget(parent){

    buttonGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    std::shared_ptr<Group> currentGroup = _points.getGroups().at(groupIndex);
    for(int j = 0; j < currentGroup->getPoints().size(); j++){
        std::shared_ptr<Point> currentPoint = currentGroup->getPoints().at(j);
        DoubleClickableButton* pointButton = new DoubleClickableButton(j, currentPoint->getName()
                                                   + " (" + QString::number(currentPoint->getPosition().getX())
                                                   + ", " + QString::number(currentPoint->getPosition().getY()) + ")", this);
        pointButton->setFlat(true);
        pointButton->setStyleSheet("text-align:left");
        buttonGroup->addButton(pointButton, j);
        layout->addWidget(pointButton);
    }
}

void PointButtonGroup::setGroup(const Points &_points, const int groupIndex){
    deleteButtons();
    std::shared_ptr<Group> currentGroup = _points.getGroups().at(groupIndex);
    for(int j = 0; j < currentGroup->getPoints().size(); j++){
        std::shared_ptr<Point> currentPoint = currentGroup->getPoints().at(j);
        DoubleClickableButton* pointButton = new DoubleClickableButton(j, currentPoint->getName()
                                                   + " (" + QString::number(currentPoint->getPosition().getX())
                                                   + ", " + QString::number(currentPoint->getPosition().getY()) + ")", this);
        pointButton->setFlat(true);
        pointButton->setStyleSheet("text-align:left");
        buttonGroup->addButton(pointButton, j);
        layout->addWidget(pointButton);
        if(currentPoint->isDisplayed())
            pointButton->setIcon(QIcon(":/icons/tick.png"));
    }
}

void PointButtonGroup::deleteButtons(void){
    while(QLayoutItem* item = layout->takeAt(0)){
        if(QWidget* button = item->widget())
            delete button;
    }
}

void PointButtonGroup::setCheckable(const bool checkable){
    foreach(QAbstractButton* button, buttonGroup->buttons())
        button->setCheckable(checkable);
}

void PointButtonGroup::uncheck(void){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}
