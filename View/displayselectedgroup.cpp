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
#include "View/buttonmenu.h"

DisplaySelectedGroup::DisplaySelectedGroup(QMainWindow *parent, std::shared_ptr<Points> const& _points) : QWidget(parent){

   VerticalScrollArea* scrollArea = new VerticalScrollArea(this);

    layout = new QVBoxLayout(this);

    points = _points;
    actionButtons = new TopLeftMenu(this);

    actionButtons->disableAll();
    actionButtons->getMinusButton()->setCheckable(false);
    actionButtons->getEditButton()->setCheckable(false);
    actionButtons->getMapButton()->setCheckable(false);

    actionButtons->getPlusButton()->setToolTip("To add a point click on the map");
    actionButtons->getMinusButton()->setToolTip("Select a point and click here to remove it");
    actionButtons->getEditButton()->setToolTip("Select a point and click here to modify it");
    actionButtons->getGoButton()->setToolTip("Select a point and click here to access its information");
    actionButtons->getMapButton()->setToolTip("Select a point and click here to display or hide it on the map");

    layout->addWidget(actionButtons);

    name = new QLabel("\nName : ", this);
    name->setStyleSheet("* {  font-weight: bold; text-decoration:underline}");


    QLabel  *label_img  = new QLabel(this);
  //  QPixmap *pixmap_img = new QPixmap(":/icons/folder.png");
    QPixmap watermark(":/icons/folder.png");

    QPixmap pixmap_img = watermark.scaled(QSize(this->width()/6,this->width()/6),  Qt::KeepAspectRatio);


         label_img->setPixmap(pixmap_img);

    QHBoxLayout *titleLayout = new QHBoxLayout;
    titleLayout->addWidget(label_img);
    titleLayout->addWidget(name);

/*
    label_img->setMaximumWidth(this->width()/4);
    label_img->setMaximumHeight(label_img->width());
    label_img->setMinimumHeight(label_img->width());*/
    titleLayout->setAlignment(Qt::AlignCenter);

    layout->addLayout(titleLayout);



    //layout->addWidget(name);

    pointButtonGroup = new PointButtonGroup(_points, 0, this);
    scrollArea->setWidget(pointButtonGroup);

    layout->addWidget(scrollArea);
    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);
    layout->setContentsMargins(0,0,0,0);
}

void DisplaySelectedGroup::setName(const QString _name){
    name->setText(_name);
    name->setWordWrap(true);
}


