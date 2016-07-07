#include "displayselectedgroup.h"
#include <View/verticalscrollarea.h>
#include <View/pointbuttongroup.h>
#include "View/spacewidget.h"
#include <QVBoxLayout>
#include <QDebug>
#include <QIcon>
#include <QPushButton>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QLabel>
#include "verticalscrollarea.h"
DisplaySelectedGroup::DisplaySelectedGroup(QMainWindow *parent, std::shared_ptr<Points> const& _points) : QWidget(parent){

   VerticalScrollArea* scrollArea = new VerticalScrollArea(this);

    layout = new QVBoxLayout(this);

    name = new QLabel("\nName : ", this);
    points = _points;
    actionButtons = new TopLeftMenu(this);

    actionButtons->disableAll();
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(false);

    actionButtons->getPlusButton()->setToolTip("To add a point click on the map");
    actionButtons->getMinusButton()->setToolTip("Select a point and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a point and click here to modify it");
    actionButtons->getEyeButton()->setToolTip("Select a point and click here to access its information");
    actionButtons->getMapButton()->setToolTip("Select a point and click here to display or hide it on the map");

    layout->addWidget(actionButtons);

    layout->addWidget(name);

    pointButtonGroup = new PointButtonGroup(_points, 0, this);
    scrollArea->setWidget(pointButtonGroup);

    layout->addWidget(scrollArea);
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
}

void DisplaySelectedGroup::setName(const QString _name){
    name->setText(_name);
    name->setWordWrap(true);
}


