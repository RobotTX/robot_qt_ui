#include "View/topleftmenu.h"
#include "spacewidget.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QHBoxLayout>

TopLeftMenu::TopLeftMenu(QWidget * parent): QWidget(parent){

/*
groupWindow = new GroupEditWindow(this);
groupWindow->getEdit()->move(200, 200);
groupWindow->getLabel()->move(300, 300);
groupWindow->hide();
*/

layout = new QVBoxLayout(this);

// GRID = + , -  , edit  buttons
plusButton = new QPushButton(QIcon(":/icons/plus.png"),"", this);
plusButton->setIconSize(parent->size());

minusButton = new QPushButton(QIcon(":/icons/minus.png"),"", this);
minusButton->setIconSize(parent->size());
/// to force the user to choose a group or point first

editButton = new QPushButton(QIcon(":/icons/edit.png"),"", this);
editButton->setIconSize(parent->size());

/// to force the user to choose a group or point first

grid = new QHBoxLayout();
grid->addWidget(plusButton);
grid->addWidget(minusButton);
grid->addWidget(editButton);

mapButton = new QPushButton(QIcon(":/icons/map.png"),"", this);
mapButton->setIconSize(parent->size());
/// to force the user to choose first


layout->addLayout(grid);

// eyeMapLayout = eye , map buttons
eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "", this);
eyeButton->setIconSize(parent->size());
/// to force the user to choose first

eyeMapLayout = new QHBoxLayout();
eyeMapLayout->addWidget(eyeButton);
eyeMapLayout->addWidget(mapButton);


layout->addLayout(eyeMapLayout);

// bar to separate with rest of class

 spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
layout->addWidget(spaceWidget);

/*
connect(plusButton, SIGNAL(clicked(bool)), parent, SLOT(plusGroupBtnEvent()));
connect(minusButton, SIGNAL(clicked(bool)), parent, SLOT(minusGroupBtnEvent()));
connect(editButton, SIGNAL(clicked(bool)), parent, SLOT(editGroupBtnEvent(bool)));
connect(eyeButton, SIGNAL(clicked()), parent, SLOT(displayPointsInGroup()));
connect(mapButton, SIGNAL(clicked()), parent, SLOT(displayGroupMapEvent()));
*/
/*
setMaximumWidth(parent->width()*4/10);
setMinimumWidth(parent->width()*4/10);
*/
//layout->setAlignment(Qt::AlignBottom);

 }

void TopLeftMenu::disableAll()
{
     plusButton->setEnabled(false);
     minusButton->setEnabled(false);
     editButton->setEnabled(false);
     eyeButton->setEnabled(false);
     mapButton->setEnabled(false);
}
void TopLeftMenu::enableAll()
{
     plusButton->setEnabled(true);
     minusButton->setEnabled(true);
     editButton->setEnabled(true);
     eyeButton->setEnabled(true);
     mapButton->setEnabled(true);
}
void TopLeftMenu::uncheckAll()
{
    plusButton->setChecked(false);
    minusButton->setChecked(false);
    editButton->setChecked(false);
    eyeButton->setChecked(false);
    mapButton->setChecked(false);
}


void TopLeftMenu::checkAll()
{
    plusButton->setChecked(true);
    minusButton->setChecked(true);
    editButton->setChecked(true);
    eyeButton->setChecked(true);
    mapButton->setChecked(true);
}


void TopLeftMenu::setAllCheckable()
{
    plusButton->setCheckable(true);
    minusButton->setCheckable(true);
    editButton->setCheckable(true);
    eyeButton->setCheckable(true);
    mapButton->setCheckable(true);
}


void TopLeftMenu::setAllNonCheckable()
{
    plusButton->setCheckable(false);
    minusButton->setCheckable(false);
    editButton->setCheckable(false);
    eyeButton->setCheckable(false);
    mapButton->setCheckable(false);
}

