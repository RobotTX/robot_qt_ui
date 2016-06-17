#include "displayselectedgroup.h"
#include <View/verticalscrollarea.h>
#include <View/pointbuttongroup.h>
#include <QVBoxLayout>
#include <QDebug>
#include <QIcon>
#include <QPushButton>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QLabel>

DisplaySelectedGroup::DisplaySelectedGroup(QMainWindow *_parent, const Points& _points)
{
    layout = new QVBoxLayout();

    name = new QLabel("\nName : ");

    backButton = new QPushButton(QIcon(":/icons/arrowLeft.png"), "Groups");
    backButton->setIconSize(_parent->size()/10);
    layout->addWidget(backButton);

    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"");
    plusButton->setIconSize(_parent->size()/10);
    /// a tool tip is a text displayed when the user moves his mouse over the button without clicking it
    plusButton->setToolTip("To add a point click on the map");
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

    mapButton = new QPushButton(QIcon(":/icons/map.png"),"");
    mapButton->setCheckable(true);
    mapButton->setIconSize(_parent->size()/10);

    eyeMapLayout = new QHBoxLayout();
    eyeMapLayout->addWidget(eyeButton);
    eyeMapLayout->addWidget(mapButton);


    layout->addLayout(grid);
    layout->addLayout(eyeMapLayout);

    layout->addWidget(name);

    scrollArea = new VerticalScrollArea();

    pointButtonGroup = new PointButtonGroup(_points, 0);

    layout->addWidget(scrollArea);

    scrollArea->setWidget(pointButtonGroup);

    setLayout(layout);
}

DisplaySelectedGroup::~DisplaySelectedGroup(){
    delete scrollArea;
    delete pointButtonGroup;
    delete backButton;
    delete plusButton;
    delete minusButton;
    delete editButton;
    delete mapButton;
    delete eyeButton;
    delete layout;
    delete eyeMapLayout;
    delete grid;
    delete name;
}

void DisplaySelectedGroup::setName(const QString _name){
    name->setText(_name);
    name->setWordWrap(true);
}
