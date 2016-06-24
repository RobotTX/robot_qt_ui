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

DisplaySelectedGroup::DisplaySelectedGroup(QMainWindow *parent, const Points& _points) : QWidget(parent){
    layout = new QVBoxLayout(this);

    name = new QLabel("\nName : ", this);

    backButton = new QPushButton(QIcon(":/icons/arrowLeft.png"), "Groups", this);
    backButton->setIconSize(parent->size()/10);
    layout->addWidget(backButton);

    plusButton = new QPushButton(QIcon(":/icons/plus.png"),"", this);
    plusButton->setIconSize(parent->size()/10);
    /// a tool tip is a text displayed when the user moves his mouse over the button without clicking it
    plusButton->setToolTip("To add a point click on the map");
    plusButton->setEnabled(false);

    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"", this);
    minusButton->setIconSize(parent->size()/10);
    minusButton->setCheckable(true);

    editButton = new QPushButton(QIcon(":/icons/edit.png"),"", this);
    editButton->setIconSize(parent->size()/10);
    editButton->setCheckable(true);

    grid = new QHBoxLayout();
    grid->addWidget(plusButton);
    grid->addWidget(minusButton);
    grid->addWidget(editButton);

    eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "", this);
    eyeButton->setIconSize(parent->size()/10);

    mapButton = new QPushButton(QIcon(":/icons/map.png"),"", this);
    mapButton->setIconSize(parent->size()/10);

    eyeMapLayout = new QHBoxLayout();
    eyeMapLayout->addWidget(eyeButton);
    eyeMapLayout->addWidget(mapButton);


    layout->addLayout(grid);
    layout->addLayout(eyeMapLayout);


    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    layout->addWidget(spaceWidget);

    layout->addWidget(name);

    scrollArea = new VerticalScrollArea(this);

    pointButtonGroup = new PointButtonGroup(_points, 0, this);

    layout->addWidget(scrollArea);

    scrollArea->setWidget(pointButtonGroup);
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
