#include "toplayout.h"
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QIcon>
#include <QDebug>
#include "View/customscrollarea.h"
#include "View/spacewidget.h"
#include <QTime>
#include <QCoreApplication>
#include "colors.h"

TopLayout::TopLayout(QMainWindow* parent): QWidget(parent), lastMessage(TEXT_COLOR_NORMAL, "")
{
   // QWidget * all = new QWidget(this);
    //QHBoxLayout* thisLayout = new QHBoxLayout(this);

   // layout = new QHBoxLayout(all);
    layout = new QHBoxLayout(this);

    menuBtn = new QPushButton(QIcon(":/icons/list.png"), "", this);
    menuBtn->setAutoDefault(true);
    menuBtn->setIconSize(parent->size()/8);
    menuBtn->setMaximumWidth(40);
    menuBtn->setFlat(true);
    menuBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    menuBtn->setStyleSheet("QPushButton:hover { background-color: "+button_hover_color+";"
                            "border: 1px solid #aaaaaa;"
                            "border-radius: 5px }");

    layout->addWidget(menuBtn);
    connect(menuBtn, SIGNAL(clicked()), parent, SLOT(openLeftMenu()));

    connectBtn = new QPushButton(QIcon(":/icons/wifi.png"), "", this);
    connectBtn->setIconSize(parent->size()/8);
    connectBtn->setMaximumWidth(40);
    connectBtn->setFlat(true);
    connectBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(connectBtn);
    connectBtn->setStyleSheet("QPushButton:hover { background-color: "+button_hover_color+";"
                            "border: 1px solid #aaaaaa;"
                            "border-radius: 5px }");
    connect(connectBtn, SIGNAL(clicked()), parent, SLOT(connectToRobot()));

    centerBtn = new QPushButton(QIcon(":/icons/center.png"), "", this);
    centerBtn->setToolTip("Click to center the map");
    centerBtn->setIconSize(parent->size()/8);
    centerBtn->setMaximumWidth(40);
    centerBtn->setFlat(true);
    centerBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(centerBtn);
    centerBtn->setStyleSheet("QPushButton:hover { background-color: "+button_hover_color+";"
                            "border: 1px solid #aaaaaa;"
                            "border-radius: 5px }");
    connect(centerBtn, SIGNAL(clicked()), parent, SLOT(centerMap()));

    settingBtn = new QPushButton(QIcon(":/icons/setting.png"), "", this);
    settingBtn->setToolTip("Click to view/edit the settings");
    settingBtn->setIconSize(parent->size()/8);
    settingBtn->setMaximumWidth(40);
    settingBtn->setFlat(true);
    settingBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(settingBtn);
    settingBtn->setStyleSheet("QPushButton:hover { background-color: "+button_hover_color+";"
                            "border: 1px solid #aaaaaa;"
                            "border-radius: 5px }");
    connect(settingBtn, SIGNAL(clicked()), parent, SLOT(settingBtnSlot()));

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
    spaceWidget->setColor("lightgrey");
    layout->addWidget(spaceWidget);

    label = new QLabel(this);
    label->setWordWrap(true);
    label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_INFO) + ";  background:transparent;}");
    label->setContentsMargins(30,0,0,0);
    layout->addWidget(label);

    CustomScrollArea* scrollArea = new CustomScrollArea(this);
    scrollArea->setWidget(label);
    layout->addWidget(scrollArea);
    label->setAutoFillBackground(true);


    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
    spaceWidget2->setColor("lightgrey");
    layout->addWidget(spaceWidget2);

    closeBtn = new QPushButton(QIcon(":/icons/close.png"), "", this);
    closeBtn->setIconSize(parent->size()/8);
    closeBtn->setMaximumWidth(40);
    closeBtn->setFlat(true);
    closeBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);

    layout->addWidget(closeBtn);
    connect(closeBtn, SIGNAL(clicked()), parent, SLOT(quit()));

    //layout->setContentsMargins(0, 0, 0, 0);
    //this->setContentsMargins(0, 0, 0, 0);




    this->setMaximumHeight(parent->height()/5);
   // this->setMaximumWidth(parent->width()*2);
   // this->setMinimumWidth(parent->width()*2);
/*
    all->setMaximumHeight(this->height());
    all->setMinimumHeight(this->height());

    all->setMaximumWidth(this->width());
    all->setMinimumWidth(this->width());
*/

   // all->setMaximumSize(parent->size());
  //  this->setMaximumHeight(parent->height()/5);
   // this->setMaximumWidth(parent->width());
   // this->setMinimumWidth(parent->width());

    this->setContentsMargins(0, 0, 0, 0);
   // thisLayout->addWidget(all);
   // this->setStyleSheet("QWidget{background-color: #5481a4}");
    QPalette pal;
    pal.setColor(QPalette::Background, top_layout_color);

    this->setPalette( pal);
    this->setAutoFillBackground(true);

}

void TopLayout::setLabel(const QString msgType, const QString msg){
    label->setText(msg);
    label->setStyleSheet("QLabel { color: " + QString(msgType) +"; background:transparent}");
}

void TopLayout::setEnable(bool enable){
    menuBtn->setEnabled(enable);
    connectBtn->setEnabled(enable);
    centerBtn->setEnabled(enable);
    closeBtn->setEnabled(enable);
    settingBtn->setEnabled(enable);
}

void TopLayout::setLabelDelay(const QString msgType, const QString msg, int delayTime){

    // if it is an error make sure the person have seen it
    if (msgType == TEXT_COLOR_DANGER)
       {
        label->setText("");
        label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_NORMAL) +";background:transparent}");
        delay(300);
        }

    // display message
    label->setText(msg);
    label->setStyleSheet("QLabel { color: " + QString(msgType) +";background:transparent}");

    // wait before to remove message
    delay(delayTime);

    // reset message
    label->setText("");
    label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_NORMAL) +";background:transparent}");


}

void TopLayout::delay(const int ms)
{
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

