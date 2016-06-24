#include "toplayout.h"
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QIcon>
#include <QDebug>
#include "View/verticalscrollarea.h"
#include "View/spacewidget.h"

TopLayout::TopLayout(QMainWindow* parent):QWidget(parent){
    layout = new QHBoxLayout(this);

    menuBtn = new QPushButton(QIcon(":/icons/list.png"), "", this);
    menuBtn->setIconSize(parent->size()/8);
    menuBtn->setMaximumWidth(40);
    menuBtn->setFlat(true);
    menuBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(menuBtn);
    connect(menuBtn, SIGNAL(clicked()), parent, SLOT(openLeftMenu()));

    connectBtn = new QPushButton(QIcon(":/icons/wifi.png"), "", this);
    connectBtn->setIconSize(parent->size()/8);
    connectBtn->setMaximumWidth(40);
    connectBtn->setFlat(true);
    connectBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(connectBtn);
    connect(connectBtn, SIGNAL(clicked()), parent, SLOT(connectToRobot()));

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
    spaceWidget->setColor("lightgrey");
    layout->addWidget(spaceWidget);

    label = new QLabel(this);
    label->setWordWrap(true);
    label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_INFO) + "}");
    label->setContentsMargins(30,0,0,0);
    //layout->addWidget(label);

    VerticalScrollArea* scrollArea = new VerticalScrollArea(this);
    scrollArea->setWidget(label);
    layout->addWidget(scrollArea);

    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
    spaceWidget2->setColor("lightgrey");
    layout->addWidget(spaceWidget2);

    closeBtn = new QPushButton(QIcon(":/icons/close.png"), "", this);
    closeBtn->setIconSize(parent->size()/8);
    closeBtn->setMaximumWidth(40);
    closeBtn->setFlat(true);
    closeBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    //closeBtn->hide();
    layout->addWidget(closeBtn);
    connect(closeBtn, SIGNAL(clicked()), parent, SLOT(quit()));


    layout->setContentsMargins(0, 0, 0, 0);

    setStyleSheet("QPushButton:hover { background-color: #efefef;"
                           "border: 1px solid #aaaaaa;"
                           "border-radius: 5px }");
    setMaximumHeight(parent->height()/5);
}

void TopLayout::setLabel(QString msgType, QString msg){
    label->setText(msg);
    label->setStyleSheet("QLabel { color: " + QString(msgType) + "}");
}

void TopLayout::disable(){
    menuBtn->setEnabled(false);
    connectBtn->setEnabled(false);
}

void TopLayout::enable(){
    menuBtn->setEnabled(true);
    connectBtn->setEnabled(true);
}
