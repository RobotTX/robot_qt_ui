#include "toplayout.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QIcon>
#include <QDebug>
#include "View/customscrollarea.h"
#include "View/spacewidget.h"
#include <QTime>
#include <QCoreApplication>
#include "stylesettings.h"
#include "View/custompushbutton.h"

TopLayout::TopLayout(QMainWindow* parent): QWidget(parent), lastMessage(TEXT_COLOR_NORMAL, ""){
    layout = new QHBoxLayout(this);

    menuBtn = new CustomPushButton(QIcon(":/icons/list.png"), "", this, CustomPushButton::ButtonType::TOP);
    menuBtn->setIconSize(xs_icon_size);
    menuBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);

    layout->addWidget(menuBtn);
    connect(menuBtn, SIGNAL(clicked()), parent, SLOT(openLeftMenu()));

    connectBtn = new CustomPushButton(QIcon(":/icons/wifi.png"), "", this, CustomPushButton::ButtonType::TOP);
    connectBtn->setIconSize(s_icon_size);
    connectBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(connectBtn);
    connect(connectBtn, SIGNAL(clicked()), parent, SLOT(connectToRobot()));

    saveMapBtn = new CustomPushButton(QIcon(":/icons/load_map.png"), "", this, CustomPushButton::ButtonType::TOP);
    saveMapBtn->setIconSize(s_icon_size);
    saveMapBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(saveMapBtn);
    saveMapBtn->setToolTip("Save the state of the map");
    connect(saveMapBtn, SIGNAL(clicked()), parent, SLOT(saveMapState()));

    centerBtn = new CustomPushButton(QIcon(":/icons/save_map.png"), "", this, CustomPushButton::ButtonType::TOP);
    centerBtn->setToolTip("Restore the state of the map");
    centerBtn->setIconSize(s_icon_size);
    centerBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(centerBtn);
    connect(centerBtn, SIGNAL(clicked()), parent, SLOT(centerMap()));

    settingBtn = new CustomPushButton(QIcon(":/icons/setting.png"), "", this, CustomPushButton::ButtonType::TOP);
    settingBtn->setToolTip("Click to view/edit the settings");
    settingBtn->setIconSize(s_icon_size);
    settingBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(settingBtn);
    connect(settingBtn, SIGNAL(clicked()), parent, SLOT(settingBtnSlot()));

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
    spaceWidget->setColor("lightgrey");
    layout->addWidget(spaceWidget);

    label = new QLabel(this);
    label->setWordWrap(true);
    label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_INFO) + ";  background:transparent;}");
    layout->addWidget(label);

    CustomScrollArea* scrollArea = new CustomScrollArea(this);
    scrollArea->setWidget(label);
    layout->addWidget(scrollArea);
    label->setAutoFillBackground(true);


    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
    spaceWidget2->setColor("lightgrey");
    layout->addWidget(spaceWidget2);

    closeBtn = new CustomPushButton(QIcon(":/icons/close.png"), "", this, CustomPushButton::ButtonType::TOP);
    closeBtn->setIconSize(xxs_icon_size);
    closeBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);

    layout->addWidget(closeBtn);
    connect(closeBtn, SIGNAL(clicked()), parent, SLOT(quit()));



    this->setMaximumHeight(top_layout_height);
    QPalette pal;
    pal.setColor(QPalette::Background, top_layout_color);

    this->setPalette( pal);
    this->setAutoFillBackground(true);

}

void TopLayout::setLabel(const QString msgType, const QString msg){
    label->setText(msg);
    label->setStyleSheet("QLabel { color: " + QString(msgType) +"; background:transparent}");
}

void TopLayout::setEnable(const bool enable){
    menuBtn->setEnabled(enable);
    connectBtn->setEnabled(enable);
    centerBtn->setEnabled(enable);
    closeBtn->setEnabled(enable);
    settingBtn->setEnabled(enable);
    saveMapBtn->setEnabled(enable);
}

void TopLayout::setLabelDelay(const QString msgType, const QString msg, int delayTime){

    /// if it is an error make sure the person have seen it
    if (msgType == TEXT_COLOR_DANGER)
       {
        label->setText("");
        label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_NORMAL) +";background:transparent}");
        delay(300);
        }

    /// display message
    label->setText(msg);
    label->setStyleSheet("QLabel { color: " + QString(msgType) +";background:transparent}");

    /// wait before to remove message
    delay(delayTime);

    /// reset message
    label->setText("");
    label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_NORMAL) +";background:transparent}");
}

void TopLayout::delay(const int ms){
    QTime dieTime= QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

