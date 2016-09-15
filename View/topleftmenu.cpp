#include "View/topleftmenu.h"
#include "spacewidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSize>
#include "View/stylesettings.h"
#include "View/custompushbutton.h"

TopLeftMenu::TopLeftMenu(QWidget * parent): QWidget(parent){
    layout = new QVBoxLayout(this);

    plusButton = new CustomPushButton(QIcon(":/icons/plus.png"), "", this, true, CustomPushButton::ButtonType::TOP_LEFT_MENU);
    plusButton->setIconSize(xs_icon_size);

    minusButton = new CustomPushButton(QIcon(":/icons/minus.png"),"", this, true, CustomPushButton::ButtonType::TOP_LEFT_MENU);
    minusButton->setIconSize(xs_icon_size);

    /// to force the user to choose a group or point first
    editButton = new CustomPushButton(QIcon(":/icons/edit.png"),"", this, true, CustomPushButton::ButtonType::TOP_LEFT_MENU);
    editButton->setIconSize(xs_icon_size);

    /// to force the user to choose a group or point first
    grid = new QHBoxLayout();
    grid->addWidget(plusButton);
    grid->addWidget(minusButton);
    grid->addWidget(editButton);
    layout->addLayout(grid);

    mapButton = new CustomPushButton(QIcon(":/icons/eye.png"),"", this, true, CustomPushButton::ButtonType::TOP_LEFT_MENU);
    mapButton->setIconSize(s_icon_size);

    goButton = new CustomPushButton(QIcon(":/icons/go_inside.png"), "", this, true, CustomPushButton::ButtonType::TOP_LEFT_MENU);
    goButton->setIconSize(s_icon_size);


    /// to force the user to choose first

    eyeMapLayout = new QHBoxLayout();
    eyeMapLayout->addWidget(goButton);
    eyeMapLayout->addWidget(mapButton);
    layout->addLayout(eyeMapLayout);


    spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    layout->addWidget(spaceWidget);

    setTabOrder(plusButton, minusButton);
    setTabOrder(minusButton, editButton);
    setTabOrder(editButton, goButton);
    setTabOrder(goButton, mapButton);

    setMaximumHeight(top_left_menu_height);
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
