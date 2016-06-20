#include "pointbuttongroup.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"
#include <QButtonGroup>
#include <QVBoxLayout>
#include <QPushButton>

PointButtonGroup::PointButtonGroup(const Points &_points, const unsigned int groupIndex)
{
    buttonGroup = new QButtonGroup();

    layout = new QVBoxLayout();
    layout->setAlignment(Qt::AlignTop);

    std::shared_ptr<Group> currentGroup = _points.getGroups().at(groupIndex);
    for(int j = 0; j < currentGroup->getPoints().size(); j++){
        std::shared_ptr<Point> currentPoint = currentGroup->getPoints().at(j);
        QPushButton* pointButton = new QPushButton(currentPoint->getName()
                                                   + " (" + QString::number(currentPoint->getPosition().getX())
                                                   + ", " + QString::number(currentPoint->getPosition().getY()) + ")");
        pointButton->setFlat(true);
        pointButton->setStyleSheet("text-align:left");
        buttonGroup->addButton(pointButton, j);
        layout->addWidget(pointButton);
    }

    setLayout(layout);
}

PointButtonGroup::~PointButtonGroup(){
    deleteButtons();
    delete layout;
    delete buttonGroup;
}

void PointButtonGroup::setGroup(const Points &_points, const int groupIndex){
    deleteButtons();
    std::shared_ptr<Group> currentGroup = _points.getGroups().at(groupIndex);
    for(int j = 0; j < currentGroup->getPoints().size(); j++){
        std::shared_ptr<Point> currentPoint = currentGroup->getPoints().at(j);
        QPushButton* pointButton = new QPushButton(currentPoint->getName()
                                                   + " (" + QString::number(currentPoint->getPosition().getX())
                                                   + ", " + QString::number(currentPoint->getPosition().getY()) + ")");
        pointButton->setFlat(true);
        pointButton->setStyleSheet("text-align:left");
        buttonGroup->addButton(pointButton, j);
        layout->addWidget(pointButton);
        if(currentPoint->isDisplayed())
            pointButton->setIcon(QIcon(":/icons/tick.png"));
        else
            pointButton->setIcon(QIcon());
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


