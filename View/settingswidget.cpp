#include "settingswidget.h"
#include <QVBoxLayout>
#include <QCheckBox>
#include <QButtonGroup>
#include <QDebug>
#include "View/robotview.h"

int SettingsWidget::currentId = 0;

SettingsWidget::SettingsWidget(QSharedPointer<Robots> robots, QWidget *parent): QWidget(parent)
{
    menuLayout = new QVBoxLayout(this);

    robotsLaserButtonGroup = new QButtonGroup(this);
    robotsLaserButtonGroup->setExclusive(false);

    for(int i = 0; i < robots->getRobotsVector().size(); i++){
        QCheckBox* activateLaserButton = new QCheckBox(robots->getRobotsVector().at(i)->getRobot()->getName());
        robotsLaserButtonGroup->addButton(activateLaserButton, currentId++);
        activateLaserButton->setChecked(true);
        menuLayout->addWidget(activateLaserButton);
    }

    connect(robotsLaserButtonGroup, SIGNAL(buttonToggled(int, bool)), this, SLOT(emitLaserSettingChange(int, bool)));
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
    robotsLaserButtonGroup->button(robotId)->hide();
    robotsLaserButtonGroup->removeButton(robotsLaserButtonGroup->button(robotId));
}
