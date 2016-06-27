#include "robots.h"
#include "View/robotview.h"
#include "robot.h"
#include <QDebug>
#include "Model/point.h"

Robots::Robots(){
    robotsVector = QVector<RobotView*>();
}


Robots::~Robots(){
    qDeleteAll(robotsVector.begin(), robotsVector.end());
    robotsVector.clear();
}

void Robots::add(RobotView* const robotView){
    robotsVector.append(robotView);
    qDebug() << "Added the robot : " << robotView->getRobot()->getName() << " to the list of robots";
}

void Robots::remove(RobotView* robotView){
    int nb = 0;
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getName() == robotView->getRobot()->getName() && robotsVector[i]->getRobot()->getIp() == robotView->getRobot()->getIp()){
            robotsVector.remove(i);
            nb++;
        }
    }
    qDebug() << nb << " robot(s) named " << robotView->getRobot()->getName() << " removed";
}

void Robots::removeByName(const QString name){
    int nb = 0;
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getName() == name){
            robotsVector.remove(i);
            nb++;
        }
    }
    qDebug() << nb << " robot(s) named " << name << " removed";
}

void Robots::removeByIp(const QString ip){
    int nb = 0;
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getIp() == ip){
            robotsVector.remove(i);
            nb++;
        }
    }
    qDebug() << nb << " robot(s) with ip address " << ip << " removed";
}

RobotView* Robots::getRobotViewByName(const QString name){
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getName() == name){
            return robotsVector[i];
        }
    }
    return new RobotView(NULL);
}

RobotView* Robots::getRobotViewByIp(const QString ip){
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getIp() == ip){
            return robotsVector[i];
        }
    }
    return new RobotView(NULL);
}

void Robots::setSelected(RobotView * const robotView){
    qDebug() << "Selected robot : " << robotView->getRobot()->getName();
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getName() == robotView->getRobot()->getName() &&
                robotsVector[i]->getRobot()->getIp() == robotView->getRobot()->getIp()){
            robotsVector[i]->setSelected(true);
        } else {
            robotsVector[i]->setSelected(false);
        }
    }
}

bool Robots::existRobotName(const QString name){
    qDebug() << "(existRobotName) Checking robot name : " << name;
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getName() == name){
            return true;
        }
    }
    return false;
}

int Robots::getRobotId(const QString name){
    qDebug() << "(getRobotId) Checking robot name : " << name;
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getName() == name){
            return i;
        }
    }
    return -1;
}

RobotView* Robots::findRobotUsingHome(const QString name) const {
    RobotView* robot(0);
    for(int i = 0; i < robotsVector.size(); i++){
        std::shared_ptr<Point> home = robotsVector.at(i)->getRobot()->getHome();
        /// we first check that this robot has a home point, if it does then we compare the names
        if(home && !home->getName().compare(name))
            return robotsVector[i];
    }
    return robot;


}
