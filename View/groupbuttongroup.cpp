#include "groupbuttongroup.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QDebug>
#include <QMouseEvent>
#include "View/doubleclickablebutton.h"

GroupButtonGroup::GroupButtonGroup(const Points &_points, QWidget* _parent):QWidget(_parent)
{
    parent = _parent;
    buttonGroup = new QButtonGroup(this);
    buttonGroup->setExclusive(true);

    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    for(int i = 0; i < _points.getGroups().size()-1; i++){
        std::shared_ptr<Group> currentGroup = _points.getGroups().at(i);
        DoubleClickableButton* groupButton = new DoubleClickableButton(i, currentGroup->getName(), this);
        groupButton->setFlat(true);
        groupButton->setStyleSheet("text-align:left");
        groupButton->setCheckable(true);
        buttonGroup->addButton(groupButton, i);
        layout->addWidget(groupButton);
        groupButton->setIconSize(_parent->size()/2);
        if(currentGroup->isDisplayed())
            groupButton->setIcon(QIcon(":/icons/folder_tick.png"));
        else
            groupButton->setIcon(QIcon(":/icons/folder.png"));
    }

    /// for the last group we just want to show the points and not "no group"
    for(int i = 0; i < _points.getGroups().at(_points.getGroups().size()-1)->getPoints().size(); i++){
        std::shared_ptr<Point> currentPoint = _points.getGroups().at(_points.getGroups().size()-1)->getPoints().at(i);
        DoubleClickableButton* pointButton = new DoubleClickableButton(i+_points.getGroups().size()-1, currentPoint->getName()
                                                   + " (" + QString::number(currentPoint->getPosition().getX())
                                                   + ", " + QString::number(currentPoint->getPosition().getY()) + ")", this);
        pointButton->setFlat(true);
        pointButton->setStyleSheet("text-align:left");
        pointButton->setCheckable(true);
        buttonGroup->addButton(pointButton, i+_points.getGroups().size()-1);
        layout->addWidget(pointButton);
        if(currentPoint->isDisplayed())
            pointButton->setIcon(QIcon(":/icons/tick.png"));
    }
}

void GroupButtonGroup::deleteButtons(void){
    while(QLayoutItem* item = layout->takeAt(0)){
        if(QWidget* button = item->widget())
            delete button;
    }
}

void GroupButtonGroup::update(const Points& _points){
    deleteButtons();
    for(int i = 0; i < _points.getGroups().size()-1; i++){
        std::shared_ptr<Group> currentGroup = _points.getGroups().at(i);
        DoubleClickableButton* groupButton = new DoubleClickableButton(i, currentGroup->getName(), this);
        groupButton->setFlat(true);
        groupButton->setStyleSheet("text-align:left");
        groupButton->setCheckable(true);
        groupButton->setIconSize(parentWidget()->size()/2);
        buttonGroup->addButton(groupButton, i);
        layout->addWidget(groupButton);
        if(currentGroup->isDisplayed())
            groupButton->setIcon(QIcon(":/icons/folder_tick.png"));
        else
            groupButton->setIcon(QIcon(":/icons/folder.png"));
    }

    /// for the last group we just want to show the points and not "no group"
    if(_points.getGroups().size() > 0){
        for(int i = 0; i < _points.getGroups().at(_points.getGroups().size()-1)->getPoints().size(); i++){
            std::shared_ptr<Point> currentPoint = _points.getGroups().at(_points.getGroups().size()-1)->getPoints().at(i);
            DoubleClickableButton* pointButton = new DoubleClickableButton(i+_points.getGroups().size()-1, currentPoint->getName()
                                                       + " (" + QString::number(currentPoint->getPosition().getX())
                                                       + ", " + QString::number(currentPoint->getPosition().getY()) + ")", this);
            pointButton->setFlat(true);
            pointButton->setStyleSheet("text-align:left");
            pointButton->setCheckable(true);
            buttonGroup->addButton(pointButton, i+_points.getGroups().size()-1);
            layout->addWidget(pointButton);
            if(currentPoint->isDisplayed())
                pointButton->setIcon(QIcon(":/icons/tick.png"));
        }
    }
    emit updateConnectionsRequest();
}

void GroupButtonGroup::uncheck(void){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

void GroupButtonGroup::mouseDoubleClickEvent(QMouseEvent *event){
    emit doubleClick(buttonGroup->checkedId());
}
