#include "View/topleftmenu.h"
#include "spacewidget.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QHBoxLayout>
#include <QSize>

TopLeftMenu::TopLeftMenu(QWidget * parent): QWidget(parent){

/*
groupWindow = new GroupEditWindow(this);
groupWindow->getEdit()->move(200, 200);
groupWindow->getLabel()->move(300, 300);
groupWindow->hide();
*/
    setMaximumHeight(parent->height()*3);

layout = new QVBoxLayout(this);
layout->setContentsMargins(0,0,0,0);
layout->setSpacing(0);

int sizeI = this->width()/3;
int sizew = (this->width()*1.6)/3;
int sizeh = this->height();
// GRID = + , -  , edit  buttons


plusButton = new QPushButton(QIcon(":/icons/plus.png"),"", this);

plusButton->setIconSize(QSize(sizeI,sizeI));
//plusButton->setMaximumHeight(sizeh);
plusButton->setMaximumWidth(sizew);
plusButton->setMinimumWidth(sizew);
plusButton->setAutoDefault(true);


/*
yourBtn->setStyleSheet("QPushButton{background:url(:/Resources/pause_nor.png);border:0px;}"
    "QPushButton:hover{background:url(:/Resources/pause_over.png);border:0px}"
    "QPushButton:pressed{background:url(:/Resources/pause_over.png); position: relative;top: 1px; left: 1px;}");
*/


//SpaceWidget* spaceWidget1 = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
//SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
//SpaceWidget* spaceWidget3 = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);



minusButton = new QPushButton(QIcon(":/icons/minus.png"),"", this);
minusButton->setIconSize(QSize(sizeI,sizeI));
minusButton->setMaximumHeight(sizeh);
minusButton->setMaximumWidth(plusButton->width());
minusButton->setMinimumWidth(plusButton->width());
minusButton->setAutoDefault(true);


/// to force the user to choose a group or point first

editButton = new QPushButton(QIcon(":/icons/edit.png"),"", this);
editButton->setIconSize(QSize(sizeI,sizeI));
editButton->setMaximumHeight(sizeh);
editButton->setMaximumWidth(plusButton->width());
editButton->setMinimumWidth(plusButton->width());
editButton->setAutoDefault(true);


/// to force the user to choose a group or point first

grid = new QHBoxLayout();
grid->setContentsMargins(0,0,0,0);
grid->setSpacing(0);


grid->addWidget(plusButton);
//grid->addWidget(spaceWidget1);
grid->addWidget(minusButton);
//grid->addWidget(spaceWidget2);
grid->addWidget(editButton);
layout->addLayout(grid);

QWidget* spaceWidget1 = new QWidget(this);
spaceWidget1->setMaximumWidth(plusButton->width()/5);
spaceWidget1->setMinimumWidth(plusButton->width()/5);
QWidget* spaceWidget2 = new QWidget(this);
spaceWidget2->setMaximumWidth(plusButton->width()/5);
spaceWidget2->setMinimumWidth(plusButton->width()/5);



mapButton = new QPushButton(QIcon(":/icons/map.png"),"", this);
mapButton->setIconSize(QSize(sizeI,sizeI));
mapButton->setMaximumHeight(sizeh);
mapButton->setMaximumWidth(sizew);
mapButton->setMinimumWidth(sizew);
mapButton->setAutoDefault(true);


// eyeMapLayout = eye , map buttons
eyeButton = new QPushButton(QIcon(":/icons/eye.png"), "", this);
eyeButton->setIconSize(QSize(sizeI,sizeI));
eyeButton->setMaximumHeight(sizeh);
eyeButton->setMaximumWidth(sizew);
eyeButton->setMinimumWidth(sizew);
eyeButton->setAutoDefault(true);


/// to force the user to choose first

eyeMapLayout = new QHBoxLayout();
eyeMapLayout->setContentsMargins(plusButton->width()/2 ,0,plusButton->width()/2,0);
eyeMapLayout->setSpacing(0);

//eyeMapLayout->addWidget(spaceWidget1);
eyeMapLayout->addWidget(eyeButton);
eyeMapLayout->addWidget(mapButton);
//eyeMapLayout->addWidget(spaceWidget2);

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



plusButton->setFlat(true);
plusButton->setStyleSheet("QPushButton{background-position: center center; border: 1px solid;       border-right-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #d3d3d3, stop: 1 transparent);border-left:none; border-top:none; border-bottom:none; position: relative;}""QPushButton:after {  content:''; background: grey;  position: absolute;  bottom: 0;  left: 0;  height: 50%; width: 1px;   }""QPushButton:hover{ background-color: grey; border: 1px;}");

minusButton->setFlat(true);
minusButton->setStyleSheet("QPushButton{background-position: center center; border: 1px solid;       border-right-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #d3d3d3, stop: 1 transparent);border-left:none; border-top:none; border-bottom:none; position: relative;}""QPushButton:after {  content:''; background: grey;  position: absolute;  bottom: 0;  left: 0;  height: 50%; width: 1px;   }""QPushButton:hover{ background-color: grey; border: 1px;}");

editButton->setFlat(true);
editButton->setStyleSheet("QPushButton{background-position: center center;}""QPushButton:hover{ background-color: grey; border: 1px;}");

eyeButton->setFlat(true);
eyeButton->setStyleSheet("QPushButton{background-position: center center; border: 1px solid;       border-right-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #d3d3d3, stop: 1 transparent);border-left:none; border-top:none; border-bottom:none; position: relative;}""QPushButton:after {  content:''; background: grey;  position: absolute;  bottom: 0;  left: 0;  height: 50%; width: 1px;   }""QPushButton:hover{ background-color: grey; border: 1px;}");
mapButton->setFlat(true);
mapButton->setStyleSheet("QPushButton{background-position: center center;}""QPushButton:hover{ background-color: grey; border: 1px;}");



setTabOrder(plusButton, minusButton);
setTabOrder(minusButton, editButton);
setTabOrder(editButton, eyeButton);
setTabOrder(eyeButton, mapButton);
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

