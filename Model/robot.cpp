#include "robot.h"
#include "Model/point.h"
#include "Model/pathpoint.h"
#include "Controller/mainwindow.h"
#include <iostream>
#include <QFile>
#include "Controller/commandcontroller.h"
#include "Model/map.h"
#include <QDir>
#include "Controller/lasercontroller.h"


Robot::Robot(MainWindow* mainWindow, const QSharedPointer<Paths>& _paths, const QString _name, const QString _ip) : QObject(mainWindow), paths(_paths), name(_name), ip(_ip), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0), pathName(""), groupName(""), sendingMap(false){

    /// we try to open the path file of the robot, if it works we do nothing otherwise we create it and put "" and ""
    /// as path name and group name for the robot
    QFile robotPathFile(QString(GOBOT_PATH) + "robots_paths/" + _name + "_path");
    if(robotPathFile.exists()){
        robotPathFile.open(QIODevice::ReadOnly);
        QDataStream in(&robotPathFile);
        in >> *this;
        robotPathFile.close();
    }
}

Robot::Robot(): name("Default name"), ip("no Ip"), position(Position()),
    orientation(0), batteryLevel(100), wifi(""), home(NULL), playingPath(0), sendingMap(false)
{}

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

    emit stopNewMapWorker();
    newMapThread.quit();
    newMapThread.wait();

    emit stopLocalMapWorker();
    localMapThread.quit();
    localMapThread.wait();

    emit stopMapWorker();
    mapThread.quit();
    mapThread.wait();

    emit stopTeleopWorker();
    teleopThread.quit();
    teleopThread.wait();
}

void Robot::portSentSlot(){
    emit startMetadataWorker();
    emit startRobotWorker();
    emit startNewMapWorker();
    emit startLocalMapWorker();
    emit startMapWorker();
    emit startTeleopWorker();
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

void Robot::sendTeleopCmd(const int cmd) {
    qDebug() << "(Robot) sendTeleopCmd" << cmd;
    emit teleopCmd(cmd);
}

void Robot::sendNewMap(QSharedPointer<Map> map) {
    qDebug() << "Robot::sendNewMap to" << name;
    if(!sendingMap){

        QString mapId = map->getMapId().toString();

        QString date = map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss");

        QString mapMetadata = QString::number(map->getWidth()) + ' ' + QString::number(map->getHeight()) +
                ' ' + QString::number(map->getResolution()) + ' ' + QString::number(map->getOrigin().getX()) +
                ' ' + QString::number(map->getOrigin().getY());

        sendingMap = 1;
        emit sendNewMapSignal(mapId, date, mapMetadata, map->getMapImage());
    } else {
        qDebug() << "Robot::sendNewMap already sending a map" << name;
    }
}

void Robot::doneSendingMapSlot(){
    sendingMap = 0;
}

void Robot::ping(){
    qDebug() << "Robot::ping" << name;
    emit pingSignal();
}

QDataStream& operator>>(QDataStream& in, Robot& robot){
    QString _pathName("");
    QString _groupName("");
    in >> _pathName >> _groupName;
    robot.setPathName(_pathName);
    robot.setGroupPathName(_groupName);
    bool flag(false);
    robot.setPath(robot.getPaths()->getPath(_groupName, _pathName, flag));
    return in;
}

QDataStream& operator<<(QDataStream& out, const Robot& robot){
    out << robot.getPathName() << robot.getGroupPathName();
    return out;
}

void Robot::clearPath(){
    pathName = "";
    groupName = "";
    path.clear();
}

void Robot::launchWorkers(MainWindow* mainWindow){

    qDebug() << "Robot" << name << "at ip" << ip << " launching its cmd thread";

    cmdRobotWorker = QPointer<CmdRobotWorker>(new CmdRobotWorker(ip, PORT_CMD, PORT_MAP_METADATA, PORT_ROBOT_POS, PORT_MAP, PORT_LOCAL_MAP, name));
    connect(cmdRobotWorker, SIGNAL(robotIsDead(QString,QString)), mainWindow, SLOT(robotIsDeadSlot(QString,QString)));
    connect(cmdRobotWorker, SIGNAL(cmdAnswer(QString)), mainWindow->getCommandController(), SLOT(cmdAnswerSlot(QString)));
    connect(cmdRobotWorker, SIGNAL(portSent()), this, SLOT(portSentSlot()));
    /// so that the first time the robot connects its home position and the last modification of its file are sent to
    /// the application in order to update the homes on both the robot and the application side correctly
    connect(cmdRobotWorker, SIGNAL(newConnection(QString, QString)), mainWindow, SLOT(updateRobotInfo(QString, QString)));
    connect(this, SIGNAL(sendCommandSignal(QString)), cmdRobotWorker, SLOT(sendCommand(QString)));
    connect(this, SIGNAL(pingSignal()), cmdRobotWorker, SLOT(pingSlot()));
    connect(mainWindow, SIGNAL(changeCmdThreadRobotName(QString)), cmdRobotWorker, SLOT(changeRobotNameSlot(QString)));
    connect(this, SIGNAL(stopCmdRobotWorker()), cmdRobotWorker, SLOT(stopWorker()));
    connect(&cmdThread, SIGNAL(finished()), cmdRobotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startCmdRobotWorker()), cmdRobotWorker, SLOT(connectSocket()));
    cmdRobotWorker->moveToThread(&cmdThread);
    cmdThread.start();

    //qDebug() << "Robot" << name << "at ip" << ip << " launching its robot pos thread at port" << PORT_ROBOT_POS;

    robotWorker = QPointer<RobotPositionWorker>(new RobotPositionWorker(ip, PORT_ROBOT_POS));
    connect(robotWorker, SIGNAL(valueChangedRobot(QString, float, float, float)),
                     mainWindow ,SLOT(updateRobot(QString, float, float, float)));
    connect(this, SIGNAL(stopRobotWorker()), robotWorker, SLOT(stopWorker()));
    connect(&robotThread, SIGNAL(finished()), robotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startRobotWorker()), robotWorker, SLOT(connectSocket()));
    robotWorker->moveToThread(&robotThread);
    robotThread.start();


    //qDebug() << "Robot" << name << "at ip" << ip << " launching its metadata thread at port" << PORT_ROBOT_POS;

    metadataWorker = QPointer<MetadataWorker>(new MetadataWorker(ip, PORT_MAP_METADATA));
    connect(metadataWorker, SIGNAL(valueChangedMetadata(int, int, float, float, float)),
                     mainWindow , SLOT(updateMetadata(int, int, float, float, float)));
    connect(this, SIGNAL(stopMetadataWorker()), metadataWorker, SLOT(stopWorker()));
    connect(this, SIGNAL(startMetadataWorker()), metadataWorker, SLOT(connectSocket()));
    connect(&metadataThread, SIGNAL(finished()), metadataWorker, SLOT(deleteLater()));
    metadataWorker->moveToThread(&metadataThread);
    metadataThread.start();


    //qDebug() << "Robot" << name << "at ip" << ip << " launching its new map thread at port" << PORT_NEW_MAP;

    newMapWorker = QPointer<SendNewMapWorker>(new SendNewMapWorker(ip, PORT_NEW_MAP));
    connect(this, SIGNAL(sendNewMapSignal(QString, QString, QString, QImage)), newMapWorker, SLOT(writeTcpDataSlot(QString, QString, QString, QImage)));
    connect(newMapWorker, SIGNAL(doneSendingNewMapSignal()), this, SLOT(doneSendingMapSlot()));
    connect(this, SIGNAL(stopNewMapWorker()), newMapWorker, SLOT(stopWorker()));
    connect(this, SIGNAL(startNewMapWorker()), newMapWorker, SLOT(connectSocket()));
    connect(&newMapThread, SIGNAL(finished()), newMapWorker, SLOT(deleteLater()));
    newMapWorker->moveToThread(&newMapThread);
    newMapThread.start();

    localMapWorker = QPointer<LocalMapWorker>(new LocalMapWorker(ip, PORT_LOCAL_MAP));
    connect(this, SIGNAL(stopLocalMapWorker()), localMapWorker, SLOT(stopWorker()));
    connect(this, SIGNAL(startLocalMapWorker()), localMapWorker, SLOT(connectSocket()));
    connect(&localMapThread, SIGNAL(finished()), localMapWorker, SLOT(deleteLater()));
    qRegisterMetaType<QVector<float>>("QVector<float>");
    connect(localMapWorker, SIGNAL(laserValues(float, float, float, QVector<float>, QString)), mainWindow->getLaserController(), SLOT(drawObstacles(float,float,float,QVector<float>,QString)));
    localMapWorker->moveToThread(&localMapThread);
    localMapThread.start();

    mapWorker = QPointer<ScanMapWorker>(new ScanMapWorker(ip, PORT_MAP, QDir::currentPath() + QDir::separator() + QString(MAP_FILE)));
    connect(mapWorker, SIGNAL(valueChangedMap(QByteArray, int, QString, QString, QString, QString, QString, QString)),
            mainWindow , SLOT(mapReceivedSlot(QByteArray, int, QString, QString, QString, QString, QString, QString)));
    connect(mapWorker, SIGNAL(newScanSaved(QString)), mainWindow , SLOT(sendNewMapToRobots(QString)));
    connect(&mapThread, SIGNAL(finished()), mapWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startMapWorker()), mapWorker, SLOT(connectSocket()));
    connect(this, SIGNAL(stopMapWorker()), mapWorker, SLOT(stopWorker()));
    mapWorker->moveToThread(&mapThread);
    mapThread.start();

    teleopWorker = QPointer<TeleopWorker>(new TeleopWorker(ip, PORT_TELEOP));
    connect(&mapThread, SIGNAL(finished()), teleopWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startTeleopWorker()), teleopWorker, SLOT(connectSocket()));
    connect(this, SIGNAL(stopTeleopWorker()), teleopWorker, SLOT(stopWorker()));
    connect(this, SIGNAL(teleopCmd(int)), teleopWorker, SLOT(writeTcpDataSlot(int)));
    teleopWorker->moveToThread(&mapThread);
    mapThread.start();

    emit startCmdRobotWorker();
}
