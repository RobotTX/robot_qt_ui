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

int SettingsWidget::currentId = 0;

SettingsWidget::SettingsWidget(QSharedPointer<Robots> robots, int settingMapChoice, QWidget *parent): QWidget(parent)
{

    setWindowTitle("Settings");
    // does not work :(
    setWindowIcon(QPixmap(":/icons/setting.png").scaled(s_icon_size));

    /// moves the page at the center of the screen
    move(QApplication::desktop()->screen()->rect().center() - rect().center());

    menuLayout = new QVBoxLayout(this);

    /**
     * LASER FEEDBACK
     * For each robot there is a checkbox, if the user wants to display the obstacles around the robot
     * in real time, he has to check the box and vice-versa
     * */

    feedBackLabel = new QLabel("Laser feedback");
    menuLayout->addWidget(feedBackLabel);

    robotsLaserButtonGroup = new QButtonGroup(this);
    robotsLaserButtonGroup->setExclusive(false);

    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        QCheckBox* activateLaserButton = new QCheckBox(robots->getRobotsVector().at(i)->getRobot()->getName());
        robotsLaserButtonGroup->addButton(activateLaserButton, currentId++);
        activateLaserButton->setChecked(true);
        menuLayout->addWidget(activateLaserButton);
    }

    connect(robotsLaserButtonGroup, SIGNAL(buttonToggled(int, bool)), this, SLOT(emitLaserSettingChange(int, bool)));

    SpaceWidget* space = new SpaceWidget(SpaceWidget::SpaceOrientation::HORIZONTAL, this);
    menuLayout->addWidget(space);

    /**
     * MAP CHOICE
     * when a robot connects, if it already contains a map the application user has
     * to decide if he wants to use the map stored on the robot or on the application
     * */
    chooseMapLabel = new QLabel("Choice of the map");
    chooseMapBox = new QComboBox(this);
    chooseMapBox->insertItem(ALWAYS_ASK, "Always ask which map I want to use");
    chooseMapBox->insertItem(ALWAYS_NEW, "Always use the newest map");
    chooseMapBox->insertItem(ALWAYS_OLD, "Always use the oldest map");
    chooseMapBox->insertItem(ALWAYS_ROBOT, "Always use the map of the robot");
    chooseMapBox->insertItem(ALWAYS_APPLICATION, "Always use the map of this application");
    chooseMapBox->setCurrentIndex(settingMapChoice);

    menuLayout->addWidget(chooseMapLabel);
    menuLayout->addWidget(chooseMapBox);
}

void SettingsWidget::emitLaserSettingChange(int robotId, bool turnOnLaserFeedBack){
    qDebug() << "laser feedback button" << robotId << "now : " << turnOnLaserFeedBack;
    emit laserFeedBack(iDtoIPMap.value(robotId), turnOnLaserFeedBack);
}

void SettingsWidget::addRobot(const QString robotIPAddress, const QString robot_name){
    qDebug() << "SettingsWidget::addRobot" << robotIPAddress << "id" << currentId;
    iDtoIPMap.insert(currentId, robotIPAddress);
    QCheckBox* activateLaserButton = new QCheckBox(robot_name);
    robotsLaserButtonGroup->addButton(activateLaserButton, currentId++);
    activateLaserButton->setChecked(true);
    menuLayout->addWidget(activateLaserButton);
}

void SettingsWidget::removeRobot(const QString robotIPAddress){
    int robotId(-1);
    QMapIterator<int, QString> it(iDtoIPMap);
    while(it.hasNext()){
        it.next();
        if(!it.value().compare(robotIPAddress)){
            robotId = it.key();
            iDtoIPMap.remove(it.key());
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
