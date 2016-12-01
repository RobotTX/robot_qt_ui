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

int SettingsWidget::currentId = 0;

SettingsWidget::SettingsWidget(QSharedPointer<Robots> robots, int _settingMapChoice, QWidget *parent)
    : QWidget(parent), settingMapChoice(_settingMapChoice){

    setWindowTitle("Settings");
    // does not work :(
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

    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        QCheckBox* activateLaserButton = new QCheckBox(robots->getRobotsVector().at(i)->getRobot()->getName(), this);
        robotsLaserButtonGroup->addButton(activateLaserButton, currentId++);
        activateLaserButton->setChecked(true);
        robotsLaserLayout->addWidget(activateLaserButton);
    }
    topLayout->addLayout(robotsLaserLayout);


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
    chooseMapBox->setCurrentIndex(settingMapChoice);

    topLayout->addWidget(chooseMapLabel);
    topLayout->addWidget(chooseMapBox);

    layout->addLayout(topLayout);


    QHBoxLayout* cancelSaveLayout = new QHBoxLayout();
    CustomPushButton* applyBtn = new CustomPushButton("Apply", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(applyBtn);
    connect(applyBtn, SIGNAL(clicked()), this, SLOT(applySlot()));

    CustomPushButton* cancelBtn = new CustomPushButton("Cancel", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(cancelBtn);
    connect(cancelBtn, SIGNAL(clicked()), this, SLOT(cancelSlot()));

    CustomPushButton* saveBtn = new CustomPushButton("Save", this, true, CustomPushButton::ButtonType::LEFT_MENU, "center");
    cancelSaveLayout->addWidget(saveBtn);
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(saveSlot()));
    layout->addLayout(cancelSaveLayout);


    topLayout->setContentsMargins(0, 0, 0, 0);
    cancelSaveLayout->setContentsMargins(0, 0, 0, 0);

    topLayout->setAlignment(Qt::AlignTop);
    cancelSaveLayout->setAlignment(Qt::AlignBottom);
}

void SettingsWidget::addRobot(const QString robotIPAddress, const QString robot_name){
    qDebug() << "SettingsWidget::addRobot" << robotIPAddress << "id" << currentId;

    // TODO get bool from settings file or something else
    idToIpMap.insert(currentId, QPair<QString, bool>(robotIPAddress, true));
    QCheckBox* activateLaserButton = new QCheckBox(robot_name, this);
    robotsLaserButtonGroup->addButton(activateLaserButton, currentId++);
    activateLaserButton->setChecked(true);
    robotsLaserLayout->addWidget(activateLaserButton);
}

void SettingsWidget::removeRobot(const QString robotIPAddress){
    int robotId(-1);
    QMapIterator<int, QPair<QString, bool>> it(idToIpMap);
    while(it.hasNext()){
        it.next();
        if(!it.value().first.compare(robotIPAddress)){
            robotId = it.key();
            idToIpMap.remove(it.key());
            break;
        }
    }
    qDebug() << "SettingsWidget::removeRobot" << robotsLaserButtonGroup->buttons().size();
    if(robotId != -1){
        robotsLaserButtonGroup->button(robotId)->hide();
        robotsLaserButtonGroup->removeButton(robotsLaserButtonGroup->button(robotId));
        delete robotsLaserButtonGroup->button(robotId);
    }
}

void SettingsWidget::applySlot(){
    qDebug() << "SettingsWidget::applySlot called";
    for(int i = 0; i < robotsLaserButtonGroup->buttons().size(); i++){
        int index = robotsLaserButtonGroup->id(robotsLaserButtonGroup->buttons().at(i));
        QString robotId = idToIpMap.value(index).first;
        bool isChecked = robotsLaserButtonGroup->buttons().at(i)->isChecked();
        if(idToIpMap.value(index).second != isChecked){
            idToIpMap.insert(index, QPair<QString, bool>(robotId, isChecked));
            emit activateLaser(idToIpMap.value(index).first, isChecked);
        }
    }
    settingMapChoice = chooseMapBox->currentIndex();

    qDebug() << "SettingsWidget::applySlot" << settingMapChoice << " : " << idToIpMap;
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
