#include "robot.h"
#include "Model/point.h"
#include "Model/pathpoint.h"
#include "Controller/cmdrobotthread.h"
#include "Controller/scanrobotthread.h"
#include "Controller/scanmetadatathread.h"
#include <QMainWindow>
#include <iostream>

Robot::Robot(const QString _name, const QString _ip, QMainWindow* parent) : name(_name), ip(_ip), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0)
{
    qDebug() << "Robot" << name << "at ip" << ip << " launching its cmd thread";

    cmdThread = new CmdRobotThread(ip, PORT_CMD, PORT_MAP_METADATA, PORT_ROBOT_POS, PORT_MAP, name, parent);
    QObject::connect(cmdThread, SIGNAL(robotIsDead(QString,QString)), parent, SLOT(robotIsDeadSlot(QString,QString)));
    QObject::connect(parent, SIGNAL(ping()), cmdThread, SLOT(pingSlot()));
    QObject::connect(parent, SIGNAL(changeCmdThreadRobotName(QString)), cmdThread, SLOT(changeRobotNameSlot(QString)));
    cmdThread->start();


    qDebug() << "Robot" << name << "at ip" << ip << " launching its robot pos thread at port" << PORT_ROBOT_POS;

    robotThread = new ScanRobotThread(ip, PORT_ROBOT_POS);
    QObject::connect(robotThread, SIGNAL(valueChangedRobot(QString, float, float, float)),
                     parent ,SLOT(updateRobot(QString, float, float, float)));
    robotThread->start();
    robotThread->moveToThread(robotThread);


    qDebug() << "Robot" << name << "at ip" << ip << " launching its metadata thread at port" << PORT_ROBOT_POS;

    metadataThread = new ScanMetadataThread(ip, PORT_MAP_METADATA);
    QObject::connect(metadataThread, SIGNAL(valueChangedMetadata(int, int, float, float, float)),
                     parent , SLOT(updateMetadata(int, int, float, float, float)));
    metadataThread->start();
    metadataThread->moveToThread(metadataThread);
}

Robot::Robot(): name("Default name"), ip("no Ip"), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0){
}

Robot::~Robot(){
    if (cmdThread != 0 && cmdThread->isRunning() ) {
        cmdThread->requestInterruption();
        cmdThread->wait();
    }
    if (robotThread != NULL && robotThread->isRunning() ) {
        robotThread->requestInterruption();
        robotThread->wait();
    }
    if (metadataThread != NULL && metadataThread->isRunning() ) {
        metadataThread->requestInterruption();
        metadataThread->wait();
    }
}

std::ostream& operator <<(std::ostream& stream, Robot const& robot){
    robot.display(stream);
    return stream;
}

void Robot::display(std::ostream& stream) const {
    stream << "Hello I am a robot called "  << getName().toStdString() << std::endl;
    stream << "My Ip address is : " << getIp().toStdString() << std::endl;
    stream << "I have " << getBatteryLevel() << "% of battery left" << std::endl;
}

bool Robot::sendCommand(const QString cmd) {
    return cmdThread->sendCommand(cmd);
    //return true;
}

QString Robot::waitAnswer() {
    return cmdThread->waitAnswer();
    //return "1 1";
}

void Robot::resetCommandAnswer() {
    cmdThread->resetCommandAnswer();
}

void Robot::stopThreads() {
    if (cmdThread != 0 && cmdThread->isRunning() ) {
        cmdThread->requestInterruption();
        cmdThread->wait();
    }
    if (robotThread != NULL && robotThread->isRunning() ) {
        robotThread->requestInterruption();
        robotThread->wait();
    }
    if (metadataThread != NULL && metadataThread->isRunning() ) {
        metadataThread->requestInterruption();
        metadataThread->wait();
    }
}
