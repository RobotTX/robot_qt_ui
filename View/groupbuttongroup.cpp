#include "groupbuttongroup.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QDebug>

GroupButtonGroup::GroupButtonGroup(const Points &_points)
{
    buttonGroup = new QButtonGroup();
    buttonGroup->setExclusive(true);

    layout = new QVBoxLayout();
    layout->setAlignment(Qt::AlignTop);

    for(int i = 0; i < _points.getGroups().size()-1; i++){
        std::shared_ptr<Group> currentGroup = _points.getGroups().at(i);
        QPushButton* groupButton = new QPushButton(currentGroup->getName());
        groupButton->setFlat(true);
        groupButton->setStyleSheet("text-align:left");
        groupButton->setCheckable(true);
        buttonGroup->addButton(groupButton, i);
        layout->addWidget(groupButton);
        if(currentGroup->isDisplayed())
            groupButton->setIcon(QIcon(":/icons/tick.png"));
    }

    /// for the last group we just want to show the points and not "no group"
    for(int i = 0; i < _points.getGroups().at(_points.getGroups().size()-1)->getPoints().size(); i++){
        std::shared_ptr<Point> currentPoint = _points.getGroups().at(_points.getGroups().size()-1)->getPoints().at(i);
        QPushButton* pointButton = new QPushButton(currentPoint->getName()
                                                   + " (" + QString::number(currentPoint->getPosition().getX())
                                                   + ", " + QString::number(currentPoint->getPosition().getY()) + ")");
        pointButton->setFlat(true);
        pointButton->setStyleSheet("text-align:left");
        pointButton->setCheckable(true);
        buttonGroup->addButton(pointButton, i+_points.getGroups().size()-1);
        layout->addWidget(pointButton);
        if(currentPoint->isDisplayed())
            pointButton->setIcon(QIcon(":/icons/tick.png"));
    }

    setLayout(layout);
}

GroupButtonGroup::~GroupButtonGroup(){
    deleteButtons();
    delete layout;
    delete buttonGroup;
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
        QPushButton* groupButton = new QPushButton(currentGroup->getName());
        groupButton->setFlat(true);
        groupButton->setStyleSheet("text-align:left");
        groupButton->setCheckable(true);
        buttonGroup->addButton(groupButton, i);
        layout->addWidget(groupButton);
    }

    /// for the last group we just want to show the points and not "no group"
    for(int i = 0; i < _points.getGroups().at(_points.getGroups().size()-1)->getPoints().size(); i++){
        std::shared_ptr<Point> currentPoint = _points.getGroups().at(_points.getGroups().size()-1)->getPoints().at(i);
        QPushButton* pointButton = new QPushButton(currentPoint->getName()
                                                   + " (" + QString::number(currentPoint->getPosition().getX())
                                                   + ", " + QString::number(currentPoint->getPosition().getY()) + ")");
        pointButton->setFlat(true);
        pointButton->setStyleSheet("text-align:left");
        pointButton->setCheckable(true);
        buttonGroup->addButton(pointButton, i+_points.getGroups().size()-1);
        layout->addWidget(pointButton);
        if(currentPoint->isDisplayed())
            pointButton->setIcon(QIcon(":/icons/tick.png"));
    }
}

void GroupButtonGroup::uncheck(void){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}
