#include "robot.h"
#include "Model/point.h"
#include "Model/pathpoint.h"
#include "Controller/cmdrobotthread.h"
#include "Controller/scanrobotthread.h"
#include "Controller/scanmetadatathread.h"
#include "Controller/sendnewmapthread.h"
#include <QMainWindow>
#include <iostream>

Robot::Robot(const QString _name, const QString _ip, QMainWindow* parent) : name(_name), ip(_ip), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0), mapId(), sendingMap(false)
{
    //qDebug() << "Robot" << name << "at ip" << ip << " launching its cmd thread";
/*
    cmdThread = new CmdRobotThread(ip, PORT_CMD, PORT_MAP_METADATA, PORT_ROBOT_POS, PORT_MAP, name, parent);
    connect(cmdThread, SIGNAL(robotIsDead(QString,QString)), parent, SLOT(robotIsDeadSlot(QString,QString)));
    connect(this, SIGNAL(sendCommandSignal(QString)), cmdThread, SLOT(sendCommand(QString)));
    connect(this, SIGNAL(pingSignal()), cmdThread, SLOT(pingSlot()));
    connect(parent, SIGNAL(changeCmdThreadRobotName(QString)), cmdThread, SLOT(changeRobotNameSlot(QString)));
    cmdThread->start();


    qDebug() << "Robot" << name << "at ip" << ip << " launching its new map thread at port" << PORT_NEW_MAP;

    newMapThread = new SendNewMapThread(ip, PORT_NEW_MAP);
    connect(this, SIGNAL(sendNewMapSignal(QByteArray)), newMapThread, SLOT(writeTcpDataSlot(QByteArray)));
    connect(newMapThread, SIGNAL(doneSendingNewMapSignal()), this, SLOT(doneSendingNewMapSlot()));
    newMapThread->start();


    qDebug() << "Robot" << name << "at ip" << ip << " launching its robot pos thread at port" << PORT_ROBOT_POS;

    robotThread = new ScanRobotThread(ip, PORT_ROBOT_POS);
    connect(robotThread, SIGNAL(valueChangedRobot(QString, float, float, float)),
                     parent ,SLOT(updateRobot(QString, float, float, float)));
    robotThread->start();
    robotThread->moveToThread(robotThread);


    qDebug() << "Robot" << name << "at ip" << ip << " launching its metadata thread at port" << PORT_ROBOT_POS;

    metadataThread = new ScanMetadataThread(ip, PORT_MAP_METADATA);
    connect(metadataThread, SIGNAL(valueChangedMetadata(int, int, float, float, float)),
                     parent , SLOT(updateMetadata(int, int, float, float, float)));
    metadataThread->start();
    metadataThread->moveToThread(metadataThread);
*/

}

Robot::Robot(): name("Default name"), ip("no Ip"), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0), mapId(), sendingMap(false){
}

Robot::~Robot(){
/*
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
    if (newMapThread != NULL && newMapThread->isRunning() ) {
        newMapThread->requestInterruption();
        newMapThread->wait();
    }
*/
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
    qDebug() << "(Robot) Send command called" << cmd;
    //emit sendCommandSignal(cmd);
   // return cmdThread->isConnected();
    return true;
}

void Robot::sendNewMap(QByteArray cmd) {
/*
    if(newMapThread->isConnected()){
        if(sendingMap){
            qDebug() << "(Robot) Send new map called but the map is already being sent";
        } else {
            qDebug() << "(Robot) Send new map called";
            sendingMap = true;
            emit sendNewMapSignal(cmd);
        }
    } else {
        qDebug() << "(Robot) The new map socket is not connected yet";
    }
*/
}

void Robot::doneSendingNewMapSlot(){
    qDebug() << "(Robot) doneSendingNewMapSlot called";
    //sendingMap = false;
}

QString Robot::waitAnswer() {
    //return cmdThread->waitAnswer();
    return "1 1";
}

void Robot::resetCommandAnswer() {
    //cmdThread->resetCommandAnswer();
}

void Robot::stopThreads() {
/*
    if (cmdThread != 0 && cmdThread->isRunning()){
        cmdThread->requestInterruption();
        cmdThread->wait();
    }
    if (robotThread != NULL && robotThread->isRunning()){
        robotThread->requestInterruption();
        robotThread->wait();
    }
    if (metadataThread != NULL && metadataThread->isRunning()){
        metadataThread->requestInterruption();
        metadataThread->wait();
    }
    if (newMapThread != NULL && newMapThread->isRunning()){
        newMapThread->requestInterruption();
        newMapThread->wait();
    }
*/
}

void Robot::ping(){
    //emit pingSignal();
}

QDataStream& operator>>(QDataStream& in, Robot& robot){
    qDebug() << "Robot operator >> called";
    /// the size of the vector has to be serialized too in order to deserialize the object correctly
    qint32 size;
    in >> size;
    qDebug() << "reconstructing a path of size" << size;
    QVector<QSharedPointer<PathPoint>> _path;
    PathPoint pathPoint;
    for(int i = 0; i < size; i++){
        in >> pathPoint;
        _path.push_back(QSharedPointer<PathPoint> (new PathPoint(pathPoint)));
    }
    robot.setPath(_path);
    return in;
}

QDataStream& operator<<(QDataStream& out, const Robot& robot){
    qDebug() << "robot operator << called with path size" << robot.getPath().size();
    qint32 pathSize(robot.getPath().size());
    out << pathSize;
    for(int i = 0; i < pathSize; i++)
        out << *(robot.getPath().at(i));
    return out;
}

void Robot::clearPath(){
    path.clear();
    //path = QVector<QSharedPointer<PathPoint>>();
}
