#include "robot.h"
#include "Model/point.h"
#include "Model/pathpoint.h"
#include "Controller/cmdrobotworker.h"
#include "Controller/robotpositionworker.h"
#include "Controller/metadataworker.h"
#include "Controller/sendnewmapworker.h"
#include "Controller/mainwindow.h"
#include <iostream>
#include <QFile>


Robot::Robot(MainWindow* mainWindow, const QSharedPointer<Paths>& _paths, const QString _name, const QString _ip) : QObject(mainWindow), name(_name), ip(_ip), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0), mapId(), pathName(""), groupName(""), sendingMap(false){

    /// we try to open the path file of the robot, if it works we do nothing otherwise we create it and put "" and ""
    /// as path name and group name for the robot
    QFile robotPathFile(QString(GOBOT_PATH) + "robots_paths/" + _name + "_path.dat");
    if(robotPathFile.exists()){
        robotPathFile.open(QIODevice::ReadOnly);
        QDataStream in(&robotPathFile);
        in >> *this;
        robotPathFile.close();
    }

    connect(this, SIGNAL(cmdAnswer(QString)), mainWindow, SLOT(cmdAnswerSlot(QString)));

    qDebug() << "Robot" << name << "at ip" << ip << " launching its cmd thread";

    cmdRobotWorker = new CmdRobotWorker(ip, PORT_CMD, PORT_MAP_METADATA, PORT_ROBOT_POS, PORT_MAP, name);
    connect(cmdRobotWorker, SIGNAL(robotIsDead(QString,QString)), mainWindow, SLOT(robotIsDeadSlot(QString,QString)));
    connect(cmdRobotWorker, SIGNAL(cmdAnswer(QString)), mainWindow, SLOT(cmdAnswerSlot(QString)));
    connect(cmdRobotWorker, SIGNAL(portSent()), this, SLOT(portSentSlot()));
    connect(this, SIGNAL(sendCommandSignal(QString)), cmdRobotWorker, SLOT(sendCommand(QString)));
    connect(this, SIGNAL(pingSignal()), cmdRobotWorker, SLOT(pingSlot()));
    connect(mainWindow, SIGNAL(changeCmdThreadRobotName(QString)), cmdRobotWorker, SLOT(changeRobotNameSlot(QString)));
    connect(this, SIGNAL(stopCmdRobotWorker()), cmdRobotWorker, SLOT(stopCmdRobotWorkerSlot()));
    connect(&cmdThread, SIGNAL(finished()), cmdRobotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startCmdRobotWorker()), cmdRobotWorker, SLOT(connectSocket()));
    cmdRobotWorker->moveToThread(&cmdThread);
    cmdThread.start();


    qDebug() << "Robot" << name << "at ip" << ip << " launching its robot pos thread at port" << PORT_ROBOT_POS;

    robotWorker = new RobotPositionWorker(ip, PORT_ROBOT_POS);
    connect(robotWorker, SIGNAL(valueChangedRobot(QString, float, float, float)),
                     mainWindow ,SLOT(updateRobot(QString, float, float, float)));
    connect(this, SIGNAL(stopRobotWorker()), robotWorker, SLOT(stopThread()));
    connect(&robotThread, SIGNAL(finished()), robotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startRobotWorker()), robotWorker, SLOT(connectSocket()));
    robotWorker->moveToThread(&robotThread);
    robotThread.start();


    qDebug() << "Robot" << name << "at ip" << ip << " launching its metadata thread at port" << PORT_ROBOT_POS;

    metadataWorker = new MetadataWorker(ip, PORT_MAP_METADATA);
    connect(metadataWorker, SIGNAL(valueChangedMetadata(int, int, float, float, float)),
                     mainWindow , SLOT(updateMetadata(int, int, float, float, float)));
    connect(this, SIGNAL(stopMetadataWorker()), metadataWorker, SLOT(stopThread()));
    connect(this, SIGNAL(startMetadataWorker()), metadataWorker, SLOT(connectSocket()));
    connect(&metadataThread, SIGNAL(finished()), metadataWorker, SLOT(deleteLater()));
    metadataWorker->moveToThread(&metadataThread);
    metadataThread.start();

/*
    qDebug() << "Robot" << name << "at ip" << ip << " launching its new map thread at port" << PORT_NEW_MAP;

    newMapWorker = new SendNewMapWorker(ip, PORT_NEW_MAP);
    connect(this, SIGNAL(sendNewMapSignal(QByteArray)), newMapWorker, SLOT(writeTcpDataSlot(QByteArray)));
    connect(this, SIGNAL(cmdAnswer(QString)), mainWindow, SLOT(cmdAnswerSlot(QString)));
    connect(newMapWorker, SIGNAL(doneSendingNewMapSignal()), this, SLOT(doneSendingMapSlot()));
    connect(this, SIGNAL(stopNewMapWorker()), newMapWorker, SLOT(stopThread()));
    connect(this, SIGNAL(startNewMapWorker()), metadataWorker, SLOT(connectSocket()));
    connect(&newMapThread, SIGNAL(finished()), newMapWorker, SLOT(deleteLater()));
    newMapWorker->moveToThread(&newMapThread);
    newMapThread.start();
*/

    emit startCmdRobotWorker();
}

Robot::Robot(): name("Default name"), ip("no Ip"), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0), mapId(), sendingMap(false){
}


Robot::~Robot(){
    stopThreads();
}

void Robot::stopThreads() {
    emit stopCmdRobotWorker();
    cmdThread.quit();
    cmdThread.wait();

    emit stopRobotWorker();
    robotThread.quit();
    robotThread.wait();

    emit stopMetadataWorker();
    metadataThread.quit();
    metadataThread.wait();
/*
    emit stopNewMapWorker();
    newMapThread.quit();
    newMapThread.wait();*/
}

void Robot::portSentSlot(){
    emit startMetadataWorker();
    emit startRobotWorker();
    emit startNewMapWorker();
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

void Robot::sendCommand(const QString cmd) {
    qDebug() << "(Robot) Send command called" << cmd;
    emit sendCommandSignal(cmd);
}

void Robot::sendNewMap(QByteArray cmd) {
    Q_UNUSED(cmd)
    qDebug() << "Robot::sendNewMap to" << name;
    if(!sendingMap){
        sendingMap = 1;
        emit sendNewMapSignal(cmd);
    }
}

void Robot::doneSendingMapSlot(){
    sendingMap = 0;
    emit cmdAnswer("Map done");
}

void Robot::ping(){
    emit pingSignal();
}

QDataStream& operator>>(QDataStream& in, Robot& robot){
    /*QString _pathName("");
    QString _groupName("");
    in >> _pathName >> _groupName;
    robot.setPathName(_pathName);
    robot.setGroupPathName(_groupName);
    bool flag(false);
    robot.setPath(robot.getPaths()->getPath(_groupName, _pathName, flag));*/
    return in;
}

QDataStream& operator<<(QDataStream& out, const Robot& robot){
    //out << robot.getPathName() << robot.getGroupPathName();
    return out;
}

void Robot::clearPath(){
    pathName = "";
    groupName = "";
    path.clear();
}
