#include "robot.h"
#include "Model/point.h"
#include "Model/pathpoint.h"
#include "Controller/cmdrobotthread.h"
#include <QMainWindow>
#include <iostream>

Robot::Robot(const QString _name, const QString _ip, const int port, QMainWindow* parent) : name(_name), ip(_ip), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0)
{
    qDebug() << "Robot" << name << "at ip" << ip << " launching its cmd thread";

    cmdThread = new CmdRobotThread(ip, port, name, parent);
    QObject::connect(cmdThread, SIGNAL(robotIsDead(QString,QString)), parent, SLOT(robotIsDeadSlot(QString,QString)));
    QObject::connect(parent, SIGNAL(ping()), cmdThread, SLOT(pingSlot()));
    cmdThread->start();

    //cmdThread->moveToThread(cmdThread);
}

Robot::Robot(): name("Default name"), ip("no Ip"), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0){
}

Robot::~Robot(){
    if (cmdThread != 0 && cmdThread->isRunning() ) {
        cmdThread->requestInterruption();
        cmdThread->wait();
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


void Robot::stopCmdThread() {
    if (cmdThread != 0 && cmdThread->isRunning() ) {
        cmdThread->requestInterruption();
        cmdThread->wait();
    }
}
