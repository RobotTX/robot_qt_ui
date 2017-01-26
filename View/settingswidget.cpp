#include "settingswidget.h"
#include <QVBoxLayout>
#include <QCheckBox>
#include <QButtonGroup>
#include <QDebug>
#include <QLabel>
#include <QComboBox>
#include "View/robotview.h"
#include <QApplication>
#include <QDesktopWidget>
#include <View/spacewidget.h>
#include <View/stylesettings.h>
#include "View/custompushbutton.h"
#include <QDir>
#include <fstream>
#include <QCheckBox>
#include "View/custompushbutton.h"


SettingsWidget::SettingsWidget(const Settings& settings, QWidget *parent): QWidget(parent){

    setWindowTitle("Settings");

    // does not work :( <- jcrois cparce que sa maman cest la main window mais sinon ca marchait me semble
    // ( ca semble marcher si la mainwindow est fermee )
    setWindowIcon(QPixmap(":/icons/setting.png").scaled(s_icon_size));

    /// moves the page at the center of the screen
    move(QApplication::desktop()->screen()->rect().center() - rect().center());

    QVBoxLayout* layout = new QVBoxLayout(this);
    QVBoxLayout* topLayout = new QVBoxLayout();
    robotsLaserLayout = new QVBoxLayout();

    /**
     * LASER FEEDBACK
     * For each robot there is a checkbox, if the user wants to display the obstacles around the robot
     * in real time, he has to check the box and vice-versa
     * */

    feedBackLabel = new QLabel("Laser feedback", this);
    topLayout->addWidget(feedBackLabel);

    robotsLaserButtonGroup = new QButtonGroup(this);
    robotsLaserButtonGroup->setExclusive(false);
    topLayout->addLayout(robotsLaserLayout);

    QMapIterator<int, QPair<QString, bool> > it(settings.getIDtoNameMap());
    while(it.hasNext()){
        it.next();
        QCheckBox* activateLaserButton = new QCheckBox(it.value().first, this);
        robotsLaserButtonGroup->addButton(activateLaserButton, settings.getCurrentId());
        activateLaserButton->setChecked(it.value().second);
        robotsLaserLayout->addWidget(activateLaserButton);
    }


    SpaceWidget* space = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    topLayout->addWidget(space);

    /**
     * MAP CHOICE
     * when a robot connects, if it already contains a map the application user has
     * to decide if he wants to use the map stored on the robot or on the application
     * */
    chooseMapLabel = new QLabel("Choice of the map", this);
    chooseMapBox = new QComboBox(this);
    chooseMapBox->insertItem(ALWAYS_ASK, "Always ask which map I want to use");
    chooseMapBox->insertItem(ALWAYS_NEW, "Always use the newest map");
    chooseMapBox->insertItem(ALWAYS_OLD, "Always use the oldest map");
    chooseMapBox->insertItem(ALWAYS_ROBOT, "Always use the map of the robot");
    chooseMapBox->insertItem(ALWAYS_APPLICATION, "Always use the map of this application");
    chooseMapBox->setCurrentIndex(settings.getSettingMapChoice());

    batteryThresholdLabel = new QLabel("Battery level warning trigger (value of the remaining battery in % under which you receive a warning)", this);
    batteryThresholdSlider = new QSlider(Qt::Horizontal, this);
    batteryThresholdSlider->setRange(0, 100);
    batteryThresholdSlider->setTickPosition(QSlider::TicksBelow);
    batteryThresholdSlider->setTickInterval(5);
    batteryThresholdSlider->setValue(settings.getBatteryWarningThreshold());

    topLayout->addWidget(chooseMapLabel);
    topLayout->addWidget(chooseMapBox);
    topLayout->addWidget(batteryThresholdLabel);
    topLayout->addWidget(batteryThresholdSlider);

    /// button to reset the display of help messages in the application
    helpButton = new CustomPushButton("Reset tutorial");
    topLayout->addWidget(helpButton);

    layout->addLayout(topLayout);

    QHBoxLayout* cancelSaveLayout = new QHBoxLayout();
    CustomPushButton* applyBtn = new CustomPushButton("Apply", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(applyBtn);
    connect(applyBtn, SIGNAL(clicked()), this, SLOT(applySlot()));

    CustomPushButton* cancelBtn = new CustomPushButton("Cancel", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(cancelBtn);
    connect(cancelBtn, SIGNAL(clicked()), this, SLOT(cancelSlot()));

    CustomPushButton* saveBtn = new CustomPushButton("Save", this, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(saveBtn);
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveSlot()));
    layout->addLayout(cancelSaveLayout);

    topLayout->setContentsMargins(0, 0, 0, 0);
    cancelSaveLayout->setContentsMargins(0, 0, 0, 0);

    topLayout->setAlignment(Qt::AlignTop);
    cancelSaveLayout->setAlignment(Qt::AlignBottom);
}

void SettingsWidget::addRobot(const QString robot_name, const int currentId, const bool laser){
    QCheckBox* activateLaserButton = new QCheckBox(robot_name, this);
    robotsLaserButtonGroup->addButton(activateLaserButton, currentId);
    activateLaserButton->setChecked(laser);
    robotsLaserLayout->addWidget(activateLaserButton);
}

void SettingsWidget::removeRobot(const int robotId){
    robotsLaserButtonGroup->button(robotId)->hide();
    robotsLaserButtonGroup->removeButton(robotsLaserButtonGroup->button(robotId));
}

void SettingsWidget::applySlot(){
    qDebug() << "SettingsWidget::applySlot called";
    emit updateMapChoice(chooseMapBox->currentIndex());
}

void SettingsWidget::cancelSlot(){
    qDebug() << "SettingsWidget::cancelSlot called";
    close();
}

void SettingsWidget::saveSlot(){
    qDebug() << "SettingsWidget::saveSlot called";
    applySlot();
    close();
}

