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

    /// defines an order for jumping from buttons using the tab key
    setTabOrder(plusButton, minusButton);
    setTabOrder(minusButton, editButton);
    setTabOrder(editButton, goButton);
    setTabOrder(goButton, mapButton);

    setMaximumHeight(top_left_menu_height);
}

void TopLeftMenu::enableAll(const bool enable){
    plusButton->setEnabled(enable);
    minusButton->setEnabled(enable);
    editButton->setEnabled(enable);
    goButton->setEnabled(enable);
    mapButton->setEnabled(enable);
}

void TopLeftMenu::checkAll(const bool check){
    plusButton->setChecked(check);
    minusButton->setChecked(check);
    editButton->setChecked(check);
    goButton->setChecked(check);
    mapButton->setChecked(check);
}

void TopLeftMenu::setCheckable(const bool checkable){
    plusButton->setCheckable(checkable);
    minusButton->setCheckable(checkable);
    editButton->setCheckable(checkable);
    goButton->setCheckable(checkable);
    mapButton->setCheckable(checkable);
}

void TopLeftMenu::setEnable(const bool enable){
    if(enable){
        for(int i = 0; i < enabledBtns.size(); i++)
            enabledBtns.at(i)->setEnabled(true);
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
