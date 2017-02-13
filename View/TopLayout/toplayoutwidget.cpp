#include "toplayoutwidget.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QIcon>
#include <QDebug>
#include "Helper/helper.h"
#include "Controller/mainwindow.h"
#include "View/Other/stylesettings.h"
#include "View/Other/custompushbutton.h"
#include "View/Other/customscrollarea.h"
#include "View/Other/spacewidget.h"

TopLayoutWidget::TopLayoutWidget(MainWindow* parent): QWidget(parent){
    QHBoxLayout* layout = new QHBoxLayout(this);

    menuBtn = new CustomPushButton(QIcon(":/icons/list.png"), "", this, CustomPushButton::ButtonType::TOP);
    menuBtn->setIconSize(xs_icon_size);
    menuBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    menuBtn->setShortcut(QKeySequence("Ctrl+M"));

    layout->addWidget(menuBtn);
    connect(menuBtn, SIGNAL(clicked()), parent, SLOT(openLeftMenu()));

    saveMapBtn = new CustomPushButton(QIcon(":/icons/load_map.png"), "", this, CustomPushButton::ButtonType::TOP);
    saveMapBtn->setIconSize(s_icon_size);
    saveMapBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(saveMapBtn);
    saveMapBtn->setToolTip("Save the configuration of the map");
    saveMapBtn->setShortcut(QKeySequence("Ctrl+K"));

    centerBtn = new CustomPushButton(QIcon(":/icons/save_map.png"), "", this, CustomPushButton::ButtonType::TOP);
    centerBtn->setToolTip("Restore the configuration of the map");
    centerBtn->setIconSize(s_icon_size);
    centerBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(centerBtn);
    centerBtn->setShortcut(QKeySequence("Ctrl+R"));
    connect(centerBtn, SIGNAL(clicked()), parent, SLOT(centerMap()));

    settingBtn = new CustomPushButton(QIcon(":/icons/setting.png"), "", this, CustomPushButton::ButtonType::TOP);
    settingBtn->setToolTip("Click to view/edit the settings");
    settingBtn->setIconSize(s_icon_size);
    settingBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(settingBtn);
    settingBtn->setShortcut(QKeySequence("Ctrl+I"));
    connect(settingBtn, SIGNAL(clicked()), parent, SLOT(settingBtnSlot()));

    testButton = new CustomPushButton(QIcon(":/icons/T_icon.png"), "", this, CustomPushButton::ButtonType::TOP);
    testButton->setToolTip("Click to try your new function");
    testButton->setIconSize(s_icon_size);
    testButton->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    layout->addWidget(testButton);
    testButton->setShortcut(QKeySequence("Ctrl+T"));
    connect(testButton, SIGNAL(clicked()), parent, SLOT(testFunctionSlot()));

    SpaceWidget* spaceWidget = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
    spaceWidget->setColor("lightgrey");
    layout->addWidget(spaceWidget);

    QWidget* widget = new QWidget(this);
    QVBoxLayout* labelLayout = new QVBoxLayout(widget);
    label = new QLabel(this);
    label->setWordWrap(true);
    label->hide();
    label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_INFO) + ";  background:transparent;}");
    labelLayout->addWidget(label);

    labelPerm = new QLabel(this);
    labelPerm->setWordWrap(true);
    labelPerm->hide();
    labelPerm->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_WARNING) + ";  background:transparent;}");
    labelLayout->addWidget(labelPerm);

    CustomScrollArea* scrollArea = new CustomScrollArea(this);
    scrollArea->setWidget(widget);
    layout->addWidget(scrollArea);
    label->setAutoFillBackground(true);

    SpaceWidget* spaceWidget2 = new SpaceWidget(SpaceWidget::SpaceOrientation::VERTICAL, this);
    spaceWidget2->setColor("lightgrey");
    layout->addWidget(spaceWidget2);

    closeBtn = new CustomPushButton(QIcon(":/icons/close.png"), "", this, CustomPushButton::ButtonType::TOP);
    closeBtn->setIconSize(xxs_icon_size);
    closeBtn->setFocusPolicy(Qt::FocusPolicy::NoFocus);

    layout->addWidget(closeBtn);
    connect(closeBtn, SIGNAL(clicked()), parent, SLOT(close()));

    this->setMaximumHeight(top_layout_height);
    QPalette pal;
    pal.setColor(QPalette::Background, top_layout_color);

    this->setPalette( pal);
    this->setAutoFillBackground(true);
}

void TopLayoutWidget::setLabel(const QString msgType, const QString msg){
    label->setText(msg);
    if(msg.isEmpty())
        label->hide();
    else
        label->show();
    label->setStyleSheet("QLabel { color: " + QString(msgType) +"; background:transparent}");
}

void TopLayoutWidget::setLabelPerm(const QString msgType, const QString msg){
    labelPerm->setText(msg);
    if(msg.isEmpty())
        labelPerm->hide();
    else
        labelPerm->show();
    labelPerm->setStyleSheet("QLabel { color: " + QString(msgType) +"; background:transparent}");
}

void TopLayoutWidget::setEnable(const bool enable){
    menuBtn->setEnabled(enable);
    centerBtn->setEnabled(enable);
    closeBtn->setEnabled(enable);
    settingBtn->setEnabled(enable);
    saveMapBtn->setEnabled(enable);
}

void TopLayoutWidget::setLabelDelay(const QString msgType, const QString msg, const int delayTime){

    if(msg.isEmpty())
        label->hide();
    else
        label->show();

    /// if it is an error make sure the person have seen it
    if (msgType == TEXT_COLOR_DANGER){
        label->setText("");
        label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_NORMAL) +";background:transparent}");
        Helper::Thread::delay(300);
    }

    /// display message
    label->setText(msg);
    label->setStyleSheet("QLabel { color: " + QString(msgType) +";background:transparent}");

    /// wait before to remove message
    Helper::Thread::delay(delayTime);

    /// reset message
    label->setText("");
    label->hide();
    label->setStyleSheet("QLabel { color: " + QString(TEXT_COLOR_NORMAL) +";background:transparent}");
}

void TopLayoutWidget::setRobotNoHomeLabel(const QString robots_string){

    if(robots_string.isEmpty())
        labelPerm->hide();
    else {
        labelPerm->show();
        setLabelPerm(TEXT_COLOR_WARNING, "\"" + robots_string + "\" do(es) not have a home point yet.\n Please choose a home for the robot(s) in the robot menu.");
    }
}
