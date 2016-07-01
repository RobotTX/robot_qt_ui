#include "View/topleftmenu.h"
#include "spacewidget.h"
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
plusButton->setToolTip("Click here to add a new group");

minusButton = new QPushButton(QIcon(":/icons/minus.png"),"", this);
minusButton->setIconSize(parent->size());
/// to force the user to choose a group or point first
minusButton->setToolTip("Select a group or a point and click here to remove it");

editButton = new QPushButton(QIcon(":/icons/edit.png"),"", this);
editButton->setIconSize(parent->size());

/// to force the user to choose a group or point first
editButton->setToolTip("Select a group or a point and click here to modify it");

grid = new QHBoxLayout();
grid->addWidget(plusButton);
grid->addWidget(minusButton);
grid->addWidget(editButton);

mapButton = new QPushButton(QIcon(":/icons/map.png"),"", this);
mapButton->setIconSize(parent->size());
/// to force the user to choose first

mapButton->setToolTip("Select a group or a point and click here to display or hide it on the map");

layout->addLayout(grid);

// eyeMapLayout = eye , map buttons
eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "", this);
eyeButton->setIconSize(parent->size());
/// to force the user to choose first
eyeButton->setToolTip("Select a group or a point and click here to display its information");

eyeMapLayout = new QHBoxLayout();
eyeMapLayout->addWidget(eyeButton);
eyeMapLayout->addWidget(mapButton);


layout->addLayout(eyeMapLayout);

// bar to separate with rest of class

SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
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
void TopLeftMenu::EnableAll()
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


void TopLeftMenu::allCheckable()
{
    plusButton->setCheckable(true);
    minusButton->setCheckable(true);
    editButton->setCheckable(true);
    eyeButton->setCheckable(true);
    mapButton->setCheckable(true);
}


void TopLeftMenu::allNonCheckable()
{
    plusButton->setCheckable(false);
    minusButton->setCheckable(false);
    editButton->setCheckable(false);
    eyeButton->setCheckable(false);
    mapButton->setCheckable(false);
}

