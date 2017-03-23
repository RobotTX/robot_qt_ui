#include "robotcontroller.h"
#include <QDir>
#include <QDebug>
#include "Helper/helper.h"
#include "Controller/Robot/cmdrobotworker.h"
#include "Controller/Robot/robotpositionworker.h"
#include "Controller/Map/metadataworker.h"
#include "Controller/Map/sendnewmapworker.h"
#include "Controller/Map/localmapworker.h"
#include "Controller/Map/scanmapworker.h"
#include "Controller/Robot/teleopworker.h"
#include "Controller/Map/particlecloudworker.h"

RobotController::RobotController(QObject *parent, QString _ip) : QObject(parent), ip(_ip){
    launchWorkers();
}

RobotController::~RobotController(){
    stopThreads();
}

void RobotController::stopThreads() {
    //qDebug() << "RobotController::stopThreads" << ip << "going to stop the threads";

    emit stopCmdRobotWorker();
    cmdThread.quit();
    cmdThread.wait();
    //qDebug() << "RobotController::stopThreads" << ip << "stopCmdRobotWorker ok";

    emit stopRobotWorker();
    robotThread.quit();
    robotThread.wait();
    //qDebug() << "RobotController::stopThreads" << ip << "stopRobotWorker ok";

    emit stopMetadataWorker();
    metadataThread.quit();
    metadataThread.wait();
    //qDebug() << "RobotController::stopThreads" << ip << "stopMetadataWorker ok";

    emit stopNewMapWorker();
    newMapThread.quit();
    newMapThread.wait();
    //qDebug() << "RobotController::stopThreads" << ip << "stopNewMapWorker ok";

    emit stopLocalMapWorker();
    localMapThread.quit();
    localMapThread.wait();
    //qDebug() << "RobotController::stopThreads" << ip << "stopLocalMapWorker ok";

    emit stopMapWorker();
    mapThread.quit();
    mapThread.wait();
    //qDebug() << "RobotController::stopThreads" << ip << "stopMapWorker ok";

    emit stopTeleopWorker();
    teleopThread.quit();
    teleopThread.wait();
    //qDebug() << "RobotController::stopThreads" << ip << "stopTeleopWorker ok";

    emit stopParticleCloudWorker();
    particleCloudThread.quit();
    particleCloudThread.wait();
    //qDebug() << "RobotController::stopThreads" << ip << "stopParticleCloudWorker ok";

    qDebug() << "RobotController::stopThreads" << ip << "all threads stopped";
}

void RobotController::portSentSlot(){
    emit startMetadataWorker();
    emit startRobotWorker();
    emit startNewMapWorker();
    emit startLocalMapWorker();
    emit startMapWorker();
    emit startTeleopWorker();
    emit startParticleCloudWorker();
}


void RobotController::launchWorkers(){

    if(ip.size() < 100)
        return;

    qDebug() << "RobotController at ip" << ip << " launching its cmd thread";

    cmdRobotWorker = QPointer<CmdRobotWorker>(new CmdRobotWorker(ip, PORT_CMD, PORT_MAP_METADATA, PORT_ROBOT_POS, PORT_MAP, PORT_LOCAL_MAP));
    connect(cmdRobotWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(cmdRobotWorker, SIGNAL(cmdAnswer(QString)), this, SLOT(cmdAnswerSlot(QString)));
    connect(cmdRobotWorker, SIGNAL(portSent()), this, SLOT(portSentSlot()));
    /// so that the first time the RobotController connects its home position and the last modification of its file are sent to
    /// the application in order to update the homes on both the RobotController and the application side correctly
    connect(cmdRobotWorker, SIGNAL(newConnection(QString, QString)), this, SLOT(updateRobotInfo(QString, QString)));
    connect(this, SIGNAL(sendCommandSignal(QString)), cmdRobotWorker, SLOT(sendCommand(QString)));
    connect(this, SIGNAL(pingSignal()), cmdRobotWorker, SLOT(pingSlot()));
    connect(this, SIGNAL(stopCmdRobotWorker()), cmdRobotWorker, SLOT(stopWorker()));
    connect(&cmdThread, SIGNAL(finished()), cmdRobotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startCmdRobotWorker()), cmdRobotWorker, SLOT(connectSocket()));
    cmdRobotWorker->moveToThread(&cmdThread);
    cmdThread.start();

    //qDebug() << "RobotController" << name << "at ip" << ip << " launching its RobotController pos thread at port" << PORT_ROBOT_POS;

    robotWorker = QPointer<RobotPositionWorker>(new RobotPositionWorker(ip, PORT_ROBOT_POS));
    connect(robotWorker, SIGNAL(valueChangedRobot(QString, float, float, float)),
                     this ,SLOT(updateRobot(QString, float, float, float)));
    connect(this, SIGNAL(stopRobotWorker()), robotWorker, SLOT(stopWorker()));
    connect(&robotThread, SIGNAL(finished()), robotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startRobotWorker()), robotWorker, SLOT(connectSocket()));
    robotWorker->moveToThread(&robotThread);
    robotThread.start();


    //qDebug() << "RobotController" << name << "at ip" << ip << " launching its metadata thread at port" << PORT_ROBOT_POS;

    metadataWorker = QPointer<MetadataWorker>(new MetadataWorker(ip, PORT_MAP_METADATA));
    connect(metadataWorker, SIGNAL(valueChangedMetadata(int, int, float, float, float)),
                     this , SLOT(updateMetadata(int, int, float, float, float)));
    connect(this, SIGNAL(stopMetadataWorker()), metadataWorker, SLOT(stopWorker()));
    connect(this, SIGNAL(startMetadataWorker()), metadataWorker, SLOT(connectSocket()));
    connect(&metadataThread, SIGNAL(finished()), metadataWorker, SLOT(deleteLater()));
    metadataWorker->moveToThread(&metadataThread);
    metadataThread.start();


    //qDebug() << "RobotController" << name << "at ip" << ip << " launching its new map thread at port" << PORT_NEW_MAP;

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
    //qRegisterMetaType<QVector<float>>("QVector<float>");
    //connect(localMapWorker, SIGNAL(laserValues(float, float, float, QVector<float>, QString)), mainWindow->getLaserController(), SLOT(drawObstacles(float,float,float,QVector<float>,QString)));
    localMapWorker->moveToThread(&localMapThread);
    localMapThread.start();

    mapWorker = QPointer<ScanMapWorker>(new ScanMapWorker(ip, PORT_MAP));
    connect(mapWorker, SIGNAL(valueChangedMap(QByteArray, int, QString, QString, QString, QString, QString, QString, int, int)),
            this , SLOT(mapReceivedSlot(QByteArray, int, QString, QString, QString, QString, QString, QString, int, int)));
    connect(mapWorker, SIGNAL(newScanSaved(QString)), this , SLOT(sendNewMapToRobots(QString)));
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

    particleCloudWorker = QPointer<ParticleCloudWorker>(new ParticleCloudWorker(ip, PORT_PARTICLE_CLOUD));
    connect(&particleCloudThread, SIGNAL(finished()), particleCloudWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startParticleCloudWorker()), particleCloudWorker, SLOT(connectSocket()));
    connect(this, SIGNAL(stopParticleCloudWorker()), particleCloudWorker, SLOT(stopWorker()));
    particleCloudWorker->moveToThread(&particleCloudThread);
    particleCloudThread.start();

    emit startCmdRobotWorker();
}

void RobotController::mapReceivedSlot(QByteArray, int, QString, QString, QString, QString, QString, QString, int, int){
    qDebug() << "RobotController::mapReceivedSlot called";
}

void RobotController::sendNewMapToRobots(QString){
    qDebug() << "RobotController::sendNewMapToRobots called";
}

void RobotController::doneSendingMapSlot(){
    qDebug() << "RobotController::doneSendingMapSlot called";
}

void RobotController::updateMetadata(int, int, float, float, float){
    qDebug() << "RobotController::updateMetadata called";
}

void RobotController::updateRobot(QString, float, float, float){
    qDebug() << "RobotController::updateRobot called";
}

void RobotController::robotIsDeadSlot(){
    qDebug() << "RobotController::robotIsDeadSlot called";
    stopThreads();
    emit robotIsDead(ip);
}

void RobotController::cmdAnswerSlot(QString){
    qDebug() << "RobotController::cmdAnswerSlot called";
}

void RobotController::updateRobotInfo(QString, QString){
    qDebug() << "RobotController::updateRobotInfo called";
}




/*
void RobotController::sendCommand(const QString cmd) {
    qDebug() << "(RobotController) Send command called" << cmd;
    emit sendCommandSignal(cmd);
}

void RobotController::sendTeleopCmd(const int cmd) {
    qDebug() << "(RobotController) sendTeleopCmd" << cmd;
    emit teleopCmd(cmd);
}

void RobotController::sendNewMap(QSharedPointer<Map> map) {
    qDebug() << "RobotController::sendNewMap to" << name;
    if(!sendingMap){

        QString mapId = map->getMapId().toString();

        QString date = map->getDateTime().toString("yyyy-MM-dd-hh-mm-ss");

        QString mapMetadata = QString::number(map->getWidth()) + ' ' + QString::number(map->getHeight()) +
                ' ' + QString::number(map->getResolution()) + ' ' + QString::number(map->getOrigin().getX()) +
                ' ' + QString::number(map->getOrigin().getY());

        sendingMap = 1;
        emit sendNewMapSignal(mapId, date, mapMetadata, map->getMapImage());
    } else {
        qDebug() << "RobotController::sendNewMap already sending a map" << name;
    }
}

void RobotController::doneSendingMapSlot(){
    sendingMap = 0;
}

void RobotController::ping(){
    //qDebug() << "RobotController::ping" << name;
    emit pingSignal();
}
*/
