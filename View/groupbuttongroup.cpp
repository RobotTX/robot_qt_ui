#include "groupbuttongroup.h"
#include "Model/points.h"
#include "Model/group.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QDebug>
#include <QMouseEvent>
#include "View/doubleclickablebutton.h"
#include "View/customizedlineedit.h"

GroupButtonGroup::GroupButtonGroup(const Points &_points, QWidget* _parent):QWidget(_parent)
{
    parent = _parent;
    buttonGroup = new QButtonGroup(this);
    //buttonGroup->setExclusive(true);

    /// to modify the name of a group
    modifyEdit = new CustomizedLineEdit(this);
    modifyEdit->setFixedWidth(1.29*modifyEdit->width());
    modifyEdit->hide();

    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    /// we are going to make this widget visible when a user wants to modify a group
    layout->addWidget(modifyEdit);
    indexModifyEdit = 0;

    for(int i = 0; i < _points.getGroups().size()-1; i++){
        std::shared_ptr<Group> currentGroup = _points.getGroups().at(i);
        DoubleClickableButton* groupButton = new DoubleClickableButton(i, currentGroup->getName(), this);
        groupButton->setFlat(true);
        groupButton->setAutoDefault(true);
        groupButton->setStyleSheet("text-align:left");
        groupButton->setCheckable(true);
        buttonGroup->addButton(groupButton, i);
        groupButton->setAutoExclusive(true);
        layout->addWidget(groupButton);
        groupButton->setIconSize(BUTTON_SIZE);
        if(currentGroup->isDisplayed())
            groupButton->setIcon(QIcon(":/icons/folder_tick.png"));
        else
            groupButton->setIcon(QIcon(":/icons/folder_space.png"));
        groupButton->setIconSize(BUTTON_SIZE);

    }

    /// for the last group we just want to show the points and not "no group"
    for(int i = 0; i < _points.getGroups().at(_points.getGroups().size()-1)->getPoints().size(); i++){
        std::shared_ptr<Point> currentPoint = _points.getGroups().at(_points.getGroups().size()-1)->getPoints().at(i);
        //DoubleClickableButton* pointButton = new DoubleClickableButton(i+_points.getGroups().size()-1, currentPoint->getName()
        //                                           + " (" + QString::number(currentPoint->getPosition().getX())
        //                                           + ", " + QString::number(currentPoint->getPosition().getY()) + ")", this);
        DoubleClickableButton* pointButton = new DoubleClickableButton(i+_points.getGroups().size()-1, currentPoint->getName(), this);
        pointButton->setAutoDefault(true);
        pointButton->setFlat(true);
        pointButton->setStyleSheet("text-align:left");
        pointButton->setCheckable(true);
        buttonGroup->addButton(pointButton, i+_points.getGroups().size()-1);
        layout->addWidget(pointButton);
        if(currentPoint->isDisplayed())
            pointButton->setIcon(QIcon(":/icons/tick_space.png"));
        else
            pointButton->setIcon(QIcon(":/icons/empty.png"));
        pointButton->setIconSize(BUTTON_SIZE);

    }
    for(int i = 0; i < buttonGroup->buttons().count()-1; i++)
        setTabOrder(buttonGroup->button(i), buttonGroup->button(i+1));
}

void GroupButtonGroup::deleteButtons(void){
    layout->removeWidget(modifyEdit);
    while(QLayoutItem* item = layout->takeAt(0)){
        if(item){
            if(QWidget* button = item->widget())
                delete button;
        } else {
            qDebug() << "oops";
        }
    }
    layout->addWidget(modifyEdit);
}

void GroupButtonGroup::update(const Points& _points){

    deleteButtons();

    for(int i = 0; i < _points.getGroups().size()-1; i++){
        if(i == indexModifyEdit){
            modifyEdit = new CustomizedLineEdit(this);
            modifyEdit->setFixedWidth(1.29*modifyEdit->width());
            layout->addWidget(modifyEdit);
            modifyEdit->hide();
        }

        std::shared_ptr<Group> currentGroup = _points.getGroups().at(i);
        DoubleClickableButton* groupButton = new DoubleClickableButton(i, currentGroup->getName(), this);
        groupButton->setFlat(true);
        groupButton->setAutoDefault(true);
        groupButton->setStyleSheet("text-align:left");
        groupButton->setCheckable(true);
        groupButton->setIconSize(BUTTON_SIZE);
        buttonGroup->addButton(groupButton, i);
        layout->addWidget(groupButton);
        if(currentGroup->isDisplayed())
            groupButton->setIcon(QIcon(":/icons/folder_tick.png"));
        else
            groupButton->setIcon(QIcon(":/icons/folder_space.png"));
        groupButton->setIconSize(BUTTON_SIZE);

    }

    /// for the last group we just want to show the points and not "no group"
    if(_points.getGroups().size() > 0){
        for(int i = 0; i < _points.getGroups().at(_points.getGroups().size()-1)->getPoints().size(); i++){
            std::shared_ptr<Point> currentPoint = _points.getGroups().at(_points.getGroups().size()-1)->getPoints().at(i);
            DoubleClickableButton* pointButton = new DoubleClickableButton(i+_points.getGroups().size()-1, currentPoint->getName()
                                                       + " (" + QString::number(currentPoint->getPosition().getX())
                                                       + ", " + QString::number(currentPoint->getPosition().getY()) + ")", this);
            pointButton->setFlat(true);
            pointButton->setAutoDefault(true);
            pointButton->setStyleSheet("text-align:left");
            pointButton->setCheckable(true);
            buttonGroup->addButton(pointButton, i+_points.getGroups().size()-1);
            layout->addWidget(pointButton);
            if(currentPoint->isDisplayed())
                pointButton->setIcon(QIcon(":/icons/tick_space.png"));
            else
                pointButton->setIcon(QIcon(":/icons/empty.png"));
            pointButton->setIconSize(BUTTON_SIZE);

        }
    }
    emit modifyEditReconnection();
    emit updateConnectionsRequest();
}

void GroupButtonGroup::uncheck(void){
    /// little trick to uncheck all buttons because the class doesn't provide a function to do it
    buttonGroup->setExclusive(false);
    if(buttonGroup->checkedButton())
        buttonGroup->checkedButton()->setChecked(false);
    buttonGroup->setExclusive(true);
}

void GroupButtonGroup::mouseDoubleClickEvent(QMouseEvent * /* unused */){
    emit doubleClick(buttonGroup->checkedId());
}

void GroupButtonGroup::setEnabled(const bool enable){
    foreach(QAbstractButton* button, buttonGroup->buttons())
        button->setEnabled(enable);
}
