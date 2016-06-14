#include "displayselectedpoint.h"
#include "Model/point.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QMainWindow>

DisplaySelectedPoint::DisplaySelectedPoint(QMainWindow *_parent)
{
    layout = new QVBoxLayout();

    backButton = new QPushButton(QIcon(":/icons/arrowLeft.png"), "Groups");
    backButton->setIconSize(_parent->size()/10);
    layout->addWidget(backButton);

    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"");
    plusButton->setIconSize(_parent->size()/10);
    plusButton->setEnabled(false);

    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"");
    minusButton->setIconSize(_parent->size()/10);
    minusButton->setCheckable(true);

    editButton = new QPushButton(QIcon(":/icons/edit.png"),"");
    editButton->setIconSize(_parent->size()/10);
    editButton->setCheckable(true);

    grid = new QHBoxLayout();
    grid->addWidget(plusButton);
    grid->addWidget(minusButton);
    grid->addWidget(editButton);

    eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "");
    eyeButton->setIconSize(_parent->size()/10);
    eyeButton->setEnabled(false);

    mapButton = new QPushButton(QIcon(":/icons/map.png"),"");
    mapButton->setCheckable(true);
    mapButton->setIconSize(_parent->size()/10);

    eyeMapLayout = new QHBoxLayout();
    eyeMapLayout->addWidget(eyeButton);
    eyeMapLayout->addWidget(mapButton);

    layout->addLayout(grid);
    layout->addLayout(eyeMapLayout);

    nameLabel = new QLabel("Name : ");
    nameLabel->setWordWrap(true);
    layout->addWidget(nameLabel);

    posXLabel = new QLabel("X : ");
    posXLabel->setWordWrap(true);
    layout->addWidget(posXLabel);

    posYLabel = new QLabel("Y : ");
    posYLabel->setWordWrap(true);
    layout->addWidget(posYLabel);
    setLayout(layout);
}

DisplaySelectedPoint::~DisplaySelectedPoint(){
    delete nameLabel;
    delete posXLabel;
    delete posYLabel;
    delete layout;
    delete backButton;
    delete plusButton;
    delete minusButton;
    delete editButton;
    delete mapButton;
    delete eyeButton;
    delete grid;
    delete eyeMapLayout;
}

void DisplaySelectedPoint::displayPointInfo(const std::shared_ptr<Point> _point){
    nameLabel->setText("Name : " + _point->getName());
    posXLabel->setText("X : " + QString::number(_point->getPosition().getX()));
    posYLabel->setText("Y : " + QString::number(_point->getPosition().getY()));
}

