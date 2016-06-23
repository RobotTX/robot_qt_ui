#include "toplayout.h"
#include <QHBoxLayout>
#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QIcon>
#include <QDebug>
#include "View/verticalscrollarea.h"

TopLayout::TopLayout(QMainWindow* parent){
    layout = new QHBoxLayout();

    menuBtn = new QPushButton(QIcon(":/icons/list.png"), "");
    menuBtn->setIconSize(parent->size()/8);
    menuBtn->setMaximumWidth(40);
    menuBtn->setFlat(true);
    menuBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(menuBtn);
    connect(menuBtn, SIGNAL(clicked()), parent, SLOT(openLeftMenu()));

    connectBtn = new QPushButton(QIcon(":/icons/wifi.png"), "");
    connectBtn->setIconSize(parent->size()/8);
    connectBtn->setMaximumWidth(40);
    connectBtn->setFlat(true);
    connectBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(connectBtn);
    connect(connectBtn, SIGNAL(clicked()), parent, SLOT(connectToRobot()));

    label = new QLabel();
    label->setWordWrap(true);
    label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_INFO) + "}");
    label->setContentsMargins(30,0,0,0);
    //layout->addWidget(label);

    VerticalScrollArea* scrollArea = new VerticalScrollArea();
    scrollArea->setWidget(label);
    scrollArea->setMaximumHeight(50);
    layout->addWidget(scrollArea);

    closeBtn = new QPushButton(QIcon(":/icons/close.png"), "");
    closeBtn->setIconSize(parent->size()/8);
    closeBtn->setMaximumWidth(40);
    closeBtn->setFlat(true);
    closeBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    closeBtn->hide();
    layout->addWidget(closeBtn);
    connect(closeBtn, SIGNAL(clicked()), parent, SLOT(quit()));

    layout->setContentsMargins(0, 0, 0, 0);

    setStyleSheet("QPushButton:hover { background-color: #efefef;"
                           "border: 1px solid #aaaaaa;"
                           "border-radius: 5px }");
    setLayout(layout);
}

TopLayout::~TopLayout(){
    delete layout;
    delete menuBtn;
    delete connectBtn;
    delete closeBtn;
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
