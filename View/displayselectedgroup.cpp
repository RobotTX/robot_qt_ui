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

DisplaySelectedGroup::DisplaySelectedGroup(QMainWindow *parent, std::shared_ptr<Points> const& _points) : QWidget(parent){
    layout = new QVBoxLayout(this);

    name = new QLabel("\nName : ", this);
    points = _points;
    actionButtons = new TopLeftMenu(this);



    actionButtons->disableAll();
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(false);







    layout->addWidget(actionButtons);


    layout->addWidget(name);

   // scrollArea = new VerticalScrollArea(this);

    pointButtonGroup = new PointButtonGroup(_points, 0, this);

  //  layout->addWidget(scrollArea);

    layout->addWidget(pointButtonGroup);
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
}

void DisplaySelectedGroup::setName(const QString _name){
    name->setText(_name);
    name->setWordWrap(true);
}


