#include "robots.h"
#include "View/Robots/robotview.h"
#include "robot.h"
#include <QDebug>
#include "Model/Points/point.h"
#include "Model/Paths/pathpoint.h"
#include "View/Points/pointview.h"

Robots::Robots(){
    robotsVector = QVector<QPointer<RobotView>>();
    robotsNameMap = QMap<QString, QString>();
}

void Robots::add(QPointer<RobotView> const robotView){
    robotsVector.append(robotView);
}

void Robots::remove(QPointer<RobotView> robotView){
    int nb(0);
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

QPointer<RobotView> Robots::getRobotViewByName(const QString name) const {
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getName() == name)
            return robotsVector[i];
    }
    return NULL;
}

QPointer<RobotView> Robots::getRobotViewByIp(const QString ip) const {
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getIp() == ip)
            return robotsVector[i];
    }
    return NULL;
}

void Robots::setSelected(QPointer<RobotView> const robotView){
    qDebug() << "Selected robot : " << robotView->getRobot()->getName();
    for(int i = 0; i < robotsVector.length(); i++){
        bool sameName = robotsVector[i]->getRobot()->getName() == robotView->getRobot()->getName() &&
                robotsVector[i]->getRobot()->getIp() == robotView->getRobot()->getIp();
        robotsVector[i]->setSelected(sameName);
    }
}

bool Robots::existRobotName(const QString name) const {
    qDebug() << "(existRobotName) Checking robot name : " << name;
    QMapIterator<QString, QString> i(robotsNameMap);
    while (i.hasNext()) {
        i.next();
        if(i.value().compare(name) == 0)
            return true;
    }
    return false;
}

int Robots::getRobotId(const QString name) const{
    for(int i = 0; i < robotsVector.length(); i++){
        if(robotsVector[i]->getRobot()->getName() == name)
            return i;
    }
    return -1;
}

QPointer<RobotView> Robots::findRobotUsingHome(const QString name) const {
    for(int i = 0; i < robotsVector.size(); i++){
        QSharedPointer<PointView> home = robotsVector.at(i)->getRobot()->getHome();
        /// we first check that this robot has a home point, if it does then we compare the names
        if(home && !home->getPoint()->getName().compare(name))
            return robotsVector[i];
    }
    return NULL;
}

QPointer<RobotView> Robots::findRobotUsingTmpPointInPath(const QSharedPointer<Point> point) const {
    for(int i = 0; i < robotsVector.size(); i++){
        QVector<QSharedPointer<PathPoint>> path = robotsVector.at(i)->getRobot()->getPath();
        for(int j = 0; j < path.size(); j++){
            if(path.at(j)->getPoint().getName().compare(point->getName()) == 0 &&
                path.at(j)->getPoint().comparePos(point->getPosition().getX(), point->getPosition().getY()))
                return robotsVector[i];
        }
    }
    return NULL;
}

/// deselects the robot on the map (changing its pixmap)
void Robots::deselect(){
    for(int i = 0; i < robotsVector.size(); i++)
        robotsVector[i]->setSelected(false);
}
