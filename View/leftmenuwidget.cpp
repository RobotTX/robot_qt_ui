#include "leftmenuwidget.h"
#include <QVBoxLayout>
#include <QMainWindow>
#include <QPushButton>

LeftMenuWidget::LeftMenuWidget(QMainWindow* parent):QWidget(parent){

    layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    /// Robot button
    QPushButton* robotBtn = new QPushButton(QIcon(":/icons/robot.png"), "Robots", this);
    robotBtn->setIconSize(parent->size()/10);
    robotBtn->setStyleSheet ("text-align: left");

    /// Point button
    QPushButton* pointBtn = new QPushButton(QIcon(":/icons/coordinates.png"), "Points", this);
    pointBtn->setIconSize(parent->size()/10);
    pointBtn->setStyleSheet ("text-align: left");

    /// Map button
    QPushButton* mapBtn = new QPushButton(QIcon(":/icons/map.png"), "Map", this);
    mapBtn->setIconSize(parent->size()/10);
    mapBtn->setStyleSheet ("text-align: left");

    layout->addWidget(robotBtn);
    layout->addWidget(pointBtn);
    layout->addWidget(mapBtn);

    connect(robotBtn, SIGNAL(clicked()), parent, SLOT(robotBtnEvent()));
    connect(pointBtn, SIGNAL(clicked()), parent, SLOT(pointBtnEvent()));
    connect(mapBtn, SIGNAL(clicked()), parent, SLOT(mapBtnEvent()));

    hide();

    setMaximumWidth(parent->width()*4/10);
    setMinimumWidth(parent->width()*4/10);

    layout->setAlignment(Qt::AlignTop);
}

