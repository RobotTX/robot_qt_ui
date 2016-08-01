#include "pointbuttongroup.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"
#include <QButtonGroup>
#include <QVBoxLayout>
#include "View/doubleclickablebutton.h"
#include <QMouseEvent>
#include <QDebug>
#include "colors.h"

PointButtonGroup::PointButtonGroup(std::shared_ptr<Points> const&_points, const int _groupIndex
                                   , QWidget* parent): QWidget(parent)
{
    groupIndex = _groupIndex;
    buttonGroup = new QButtonGroup(this);
    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    std::shared_ptr<Group> currentGroup = _points->getGroups().at(groupIndex);
    for(int j = 0; j < currentGroup->getPoints().size(); j++){
        std::shared_ptr<Point> currentPoint = currentGroup->getPoints().at(j);
        DoubleClickableButton* pointButton = new DoubleClickableButton(j, currentPoint->getName(), this);
        pointButton->setAutoDefault(true);
        pointButton->setFlat(true);
        pointButton->setStyleSheet("text-align:left");
        buttonGroup->addButton(pointButton, j);
        layout->addWidget(pointButton);
        if(currentPoint->isDisplayed())
            pointButton->setIcon(QIcon(":/icons/eye_point.png"));
        else
            pointButton->setIcon(QIcon(":/icons/space_point.png"));
        BUTTON_SIZE = parentWidget()->size()/2 ;
        pointButton->setIconSize(BUTTON_SIZE);

    }
}

void PointButtonGroup::setGroup(std::shared_ptr<Points> const&_points, const int _groupIndex){
    deleteButtons();
    groupIndex = _groupIndex;
    std::shared_ptr<Group> currentGroup = _points->getGroups().at(groupIndex);
    for(int j = 0; j < currentGroup->getPoints().size(); j++){
        std::shared_ptr<Point> currentPoint = currentGroup->getPoints().at(j);
        DoubleClickableButton* pointButton = new DoubleClickableButton(j, currentPoint->getName(), this);

        pointButton->setAutoDefault(true);
        pointButton->setFlat(true);
        pointButton->setStyleSheet("QPushButton {color: "+text_color+";text-align:left;border: 4px; padding: 10px;}QPushButton:hover{background-color: "+button_hover_color+";}QPushButton:checked{background-color: "+button_checked_color+";}");

        buttonGroup->addButton(pointButton, j);
        layout->addWidget(pointButton);
        if(currentPoint->isDisplayed())
            pointButton->setIcon(QIcon(":/icons/eye_point.png"));
        else
            pointButton->setIcon(QIcon(":/icons/space_point.png"));
        pointButton->setIconSize(BUTTON_SIZE);
    }
    emit updateConnectionsRequest();
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
