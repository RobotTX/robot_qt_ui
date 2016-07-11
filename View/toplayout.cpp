#include "toplayout.h"
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QIcon>
#include <QDebug>
#include "View/verticalscrollarea.h"
#include "View/spacewidget.h"
#include <QTime>
 #include <QCoreApplication>

TopLayout::TopLayout(QMainWindow* parent):QWidget(parent){
    layout = new QHBoxLayout(this);

    menuBtn = new QPushButton(QIcon(":/icons/list.png"), "", this);
    menuBtn->setAutoDefault(true);
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

//  centerBtn = new QPushButton(QIcon(":/icons/"))

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

void TopLayout::setLabel(const QString msgType, const QString msg){
    label->setText(msg);
    label->setStyleSheet("QLabel { color: " + QString(msgType) + "}");
}

void TopLayout::setLabelDelay(const QString msgType, const QString msg, int delayTime){

    // if it is an error make sure the person have seen it
    if (msgType == TEXT_COLOR_DANGER)
       {
        label->setText("");
        label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_NORMAL) + "}");
        delay(300);
        }

    // display message
    label->setText(msg);
    label->setStyleSheet("QLabel { color: " + QString(msgType) + "}");

    // wait before to remove message
    delay(delayTime);

    // reset message
    label->setText("");
    label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_NORMAL) + "}");


}

void TopLayout::delay(const int ms)
{
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void TopLayout::disable(){
    menuBtn->setEnabled(false);
    connectBtn->setEnabled(false);
}

void TopLayout::enable(){
    menuBtn->setEnabled(true);
    connectBtn->setEnabled(true);
}
