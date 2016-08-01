#include "View/topleftmenu.h"
#include "spacewidget.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <QHBoxLayout>
#include <QSize>
#include <buttonmenu.h>
#include "colors.h"

TopLeftMenu::TopLeftMenu(QWidget * parent): QWidget(parent){

    setMaximumHeight(parent->height()*3);

    layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    layout->setSpacing(0);

    int sizeI = this->width()/3;
    int sizew = (this->width()*1.6)/3;
    int sizeh = this->height();


    plusButton = new ButtonMenu(QIcon(":/icons/plus.png"),"", this);

    plusButton->setIconSize(QSize(sizeI,sizeI));
    plusButton->setMaximumWidth(sizew);
    plusButton->setMinimumWidth(sizew);
    plusButton->setAutoDefault(true);


    minusButton = new QPushButton(QIcon(":/icons/minus.png"),"", this);
    minusButton->setIconSize(QSize(sizeI,sizeI));
    minusButton->setMaximumHeight(sizeh);
    minusButton->setMaximumWidth(plusButton->width());
    minusButton->setMinimumWidth(plusButton->width());
    minusButton->setAutoDefault(true);


    /// to force the user to choose a group or point first

    editButton = new ButtonMenu(QIcon(":/icons/edit.png"),"", this);
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
    grid->addWidget(minusButton);
    grid->addWidget(editButton);
    layout->addLayout(grid);

    QWidget* spaceWidget1 = new QWidget(this);
    spaceWidget1->setMaximumWidth(plusButton->width()/5);
    spaceWidget1->setMinimumWidth(plusButton->width()/5);
    QWidget* spaceWidget2 = new QWidget(this);
    spaceWidget2->setMaximumWidth(plusButton->width()/5);
    spaceWidget2->setMinimumWidth(plusButton->width()/5);

    mapButton = new QPushButton(QIcon(":/icons/eye.png"),"", this);
    mapButton->setIconSize(QSize(sizeI,sizeI));
    mapButton->setMaximumHeight(sizeh);
    mapButton->setMaximumWidth(sizew);
    mapButton->setMinimumWidth(sizew);
    mapButton->setAutoDefault(true);

    goButton = new QPushButton(QIcon(":/icons/go_inside.png"), "", this);
    goButton->setIconSize(QSize(sizeI,sizeI));
    goButton->setMaximumHeight(sizeh);
    goButton->setMaximumWidth(sizew);
    goButton->setMinimumWidth(sizew);
    goButton->setAutoDefault(true);


    /// to force the user to choose first

    eyeMapLayout = new QHBoxLayout();
    eyeMapLayout->setContentsMargins(plusButton->width()/2 ,0,plusButton->width()/2,0);
    eyeMapLayout->setSpacing(0);
    eyeMapLayout->addWidget(goButton);
    eyeMapLayout->addWidget(mapButton);
    layout->addLayout(eyeMapLayout);


    spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    layout->addWidget(spaceWidget);


    plusButton->setFlat(true);

    minusButton->setFlat(true);
    minusButton->setStyleSheet("QPushButton{background-position: center center; border: 1px solid;       border-right-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #d3d3d3, stop: 1 transparent);border-left:none; border-top:none; border-bottom:none; position: relative;}QPushButton:after {  content:''; background: grey;  position: absolute;  bottom: 0;  left: 0;  height: 50%; width: 1px;   }QPushButton:hover{ background-color: "+button_hover_color+";}");

    editButton->setFlat(true);
    editButton->setStyleSheet("QPushButton{background-position: center center;border: 1px;}""QPushButton:hover{ background-color: grey; }");

    goButton->setFlat(true);
    goButton->setStyleSheet("QPushButton{background-position: center center; border: 1px solid;       border-right-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, x3: 0, y3: 0, stop: 0 transparent, stop: 0.5 #d3d3d3, stop: 1 transparent);border-left:none; border-top:none; border-bottom:none; position: relative;}QPushButton:after {  content:''; background: grey;  position: absolute;  bottom: 0;  left: 0;  height: 50%; width: 1px;   }QPushButton:hover{ background-color: "+button_hover_color+";}");

    mapButton->setFlat(true);
    mapButton->setStyleSheet("QPushButton{background-position: center center;border: 1px;}""QPushButton:hover{ background-color: "+button_hover_color+";}QPushButton:checked{ background-color: "+button_checked_color+";}");


    setTabOrder(plusButton, minusButton);
    setTabOrder(minusButton, editButton);
    setTabOrder(editButton, goButton);
    setTabOrder(goButton, mapButton);
}

void TopLeftMenu::disableAll(){
    plusButton->setEnabled(false);
    minusButton->setEnabled(false);
    editButton->setEnabled(false);
    goButton->setEnabled(false);
    mapButton->setEnabled(false);
}
void TopLeftMenu::enableAll(){
    plusButton->setEnabled(true);
    minusButton->setEnabled(true);
    editButton->setEnabled(true);
    goButton->setEnabled(true);
    mapButton->setEnabled(true);
}
void TopLeftMenu::uncheckAll(){
    plusButton->setChecked(false);
    minusButton->setChecked(false);
    editButton->setChecked(false);
    goButton->setChecked(false);
    mapButton->setChecked(false);
}


void TopLeftMenu::checkAll(){
    plusButton->setChecked(true);
    minusButton->setChecked(true);
    editButton->setChecked(true);
    goButton->setChecked(true);
    mapButton->setChecked(true);
}

void TopLeftMenu::setAllCheckable(){
    plusButton->setCheckable(true);
    minusButton->setCheckable(true);
    editButton->setCheckable(true);
    goButton->setCheckable(true);
    mapButton->setCheckable(true);
}

void TopLeftMenu::setAllNonCheckable(){
    plusButton->setCheckable(false);
    minusButton->setCheckable(false);
    editButton->setCheckable(false);
    goButton->setCheckable(false);
    mapButton->setCheckable(false);
}

void TopLeftMenu::setEnable(bool enable){
    if(enable){

        for(int i =0; i < enabledBtns.size(); i++){
            enabledBtns.at(i)->setEnabled(true);
        }
        enabledBtns.clear();

    } else {
        if(plusButton->isEnabled()){
            plusButton->setEnabled(false);
            enabledBtns.push_back(plusButton);
        }

        if(minusButton->isEnabled()){
            minusButton->setEnabled(false);
            enabledBtns.push_back(minusButton);
        }

        if(editButton->isEnabled()){
            editButton->setEnabled(false);
            enabledBtns.push_back(editButton);
        }

        if(goButton->isEnabled()){
            goButton->setEnabled(false);
            enabledBtns.push_back(goButton);
        }

        if(mapButton->isEnabled()){
            mapButton->setEnabled(false);
            enabledBtns.push_back(mapButton);
        }
    }
}
