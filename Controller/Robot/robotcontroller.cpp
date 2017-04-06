#include "robotcontroller.h"
#include <QDir>
#include <QDebug>
#include "Helper/helper.h"
#include "Controller/Robot/robotscontroller.h"
#include "Controller/Robot/cmdrobotworker.h"
#include "Controller/Robot/robotpositionworker.h"
#include "Controller/Robot/teleopworker.h"
#include "Controller/Robot/commandcontroller.h"
#include "Controller/Map/metadataworker.h"
#include "Controller/Map/sendnewmapworker.h"
#include "Controller/Map/localmapworker.h"
#include "Controller/Map/scanmapworker.h"
#include "Controller/Map/particlecloudworker.h"

RobotController::RobotController(RobotsController *parent, QString _ip):
    QObject(parent), ip(_ip), sendingMap(false), commandController(QPointer<CommandController>(new CommandController(this, ip))){

    connect(commandController, SIGNAL(updateName(QString,QString)), parent, SLOT(updateNameSlot(QString,QString)));
    connect(commandController, SIGNAL(updateHome(QString,QString,float,float)), parent, SLOT(updateHomeSlot(QString,QString,float,float)));
    connect(commandController, SIGNAL(updatePath(QString, QStringList)), parent, SLOT(updatePathSlot(QString, QStringList)));
    connect(commandController, SIGNAL(stoppedDeletedPath(QString)), parent, SLOT(stoppedDeletedPathSlot(QString)));
    connect(commandController, SIGNAL(updatePlayingPath(QString, bool)), parent, SLOT(updatePlayingPathSlot(QString, bool)));
    connect(commandController, SIGNAL(startedScanning(QString)), parent, SLOT(startedScanningSlot(QString)));
    connect(commandController, SIGNAL(stoppedScanning(QString)), parent, SLOT(stoppedScanningSlot(QString)));
    connect(commandController, SIGNAL(playedScanning(QString)), parent, SLOT(startedScanningSlot(QString)));
    connect(commandController, SIGNAL(pausedScanning(QString)), parent, SLOT(pausedScanningSlot(QString)));

    connect(this, SIGNAL(robotIsDead(QString)), parent, SLOT(robotIsDeadSlot(QString)));
    connect(this, SIGNAL(newRobotPos(QString, float, float, float)), parent, SLOT(newRobotPosSlot(QString, float, float, float)));
    connect(this, SIGNAL(newMetadata(int, int, float, float, float)), parent, SLOT(newMetadataSlot(int, int, float, float, float)));
    connect(this, SIGNAL(updatePath(QString, QStringList)), parent, SLOT(updatePathSlot(QString, QStringList)));
    connect(this, SIGNAL(updateHome(QString, QString, float, float)), parent, SLOT(updateHomeSlot(QString, QString, float, float)));
    connect(this, SIGNAL(checkMapInfo(QString, QString, QString)), parent, SLOT(checkMapInfoSlot(QString, QString, QString)));
    connect(this, SIGNAL(newMapFromRobot(QString, QByteArray, QString, QString)), parent, SLOT(newMapFromRobotSlot(QString, QByteArray, QString, QString)));
    connect(this, SIGNAL(mapToMergeFromRobot(QByteArray, QString)), parent, SLOT(processMapForMerge(QByteArray, QString)));
    connect(this, SIGNAL(receivedScanMap(QString, QByteArray, QString)), parent, SLOT(receivedScanMapSlot(QString, QByteArray, QString)));
    connect(this, SIGNAL(checkScanning(QString, bool)), parent, SLOT(checkScanningSlot(QString, bool)));

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

    if(ip.length() < 3)
        return;

    qDebug() << "RobotController at ip" << ip << " launching its cmd thread";

    cmdRobotWorker = QPointer<CmdRobotWorker>(new CmdRobotWorker(ip, PORT_CMD, PORT_MAP_METADATA, PORT_ROBOT_POS, PORT_MAP, PORT_LOCAL_MAP));
    connect(cmdRobotWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(cmdRobotWorker, SIGNAL(cmdAnswer(QString)), commandController, SLOT(cmdAnswerSlot(QString)));
    connect(cmdRobotWorker, SIGNAL(portSent()), this, SLOT(portSentSlot()));
    /// so that the first time the RobotController connects its home position and the last modification of its file are sent to
    /// the application in order to update the homes on both the RobotController and the application side correctly
    connect(cmdRobotWorker, SIGNAL(newConnection(QString)), this, SLOT(updateRobotInfo(QString)));
    connect(commandController, SIGNAL(sendCommandSignal(QString)), cmdRobotWorker, SLOT(sendCommand(QString)));
    connect(this, SIGNAL(pingSignal()), cmdRobotWorker, SLOT(pingSlot()));
    connect(this, SIGNAL(stopCmdRobotWorker()), cmdRobotWorker, SLOT(stopWorker()));
    connect(&cmdThread, SIGNAL(finished()), cmdRobotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startCmdRobotWorker()), cmdRobotWorker, SLOT(connectSocket()));
    cmdRobotWorker->moveToThread(&cmdThread);
    cmdThread.start();

    //qDebug() << "RobotController" << name << "at ip" << ip << " launching its RobotController pos thread at port" << PORT_ROBOT_POS;

    robotWorker = QPointer<RobotPositionWorker>(new RobotPositionWorker(ip, PORT_ROBOT_POS));
    connect(robotWorker, SIGNAL(valueChangedRobot(float, float, float)),
                     this ,SLOT(updateRobot(float, float, float)));
    connect(robotWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(this, SIGNAL(stopRobotWorker()), robotWorker, SLOT(stopWorker()));
    connect(&robotThread, SIGNAL(finished()), robotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startRobotWorker()), robotWorker, SLOT(connectSocket()));
    robotWorker->moveToThread(&robotThread);
    robotThread.start();

    //qDebug() << "RobotController" << name << "at ip" << ip << " launching its metadata thread at port" << PORT_ROBOT_POS;

    metadataWorker = QPointer<MetadataWorker>(new MetadataWorker(ip, PORT_MAP_METADATA));
    connect(metadataWorker, SIGNAL(valueChangedMetadata(int, int, float, float, float)),
                     this , SLOT(updateMetadata(int, int, float, float, float)));
    connect(metadataWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(this, SIGNAL(stopMetadataWorker()), metadataWorker, SLOT(stopWorker()));
    connect(this, SIGNAL(startMetadataWorker()), metadataWorker, SLOT(connectSocket()));
    connect(&metadataThread, SIGNAL(finished()), metadataWorker, SLOT(deleteLater()));
    metadataWorker->moveToThread(&metadataThread);
    metadataThread.start();

    //qDebug() << "RobotController" << name << "at ip" << ip << " launching its new map thread at port" << PORT_NEW_MAP;

    newMapWorker = QPointer<SendNewMapWorker>(new SendNewMapWorker(ip, PORT_NEW_MAP));
    connect(this, SIGNAL(sendNewMapSignal(QString, QString, QString, QImage)), newMapWorker, SLOT(writeTcpDataSlot(QString, QString, QString, QImage)));
    connect(newMapWorker, SIGNAL(doneSendingNewMapSignal()), this, SLOT(doneSendingMapSlot()));
    connect(newMapWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(this, SIGNAL(stopNewMapWorker()), newMapWorker, SLOT(stopWorker()));
    connect(this, SIGNAL(startNewMapWorker()), newMapWorker, SLOT(connectSocket()));
    connect(&newMapThread, SIGNAL(finished()), newMapWorker, SLOT(deleteLater()));
    newMapWorker->moveToThread(&newMapThread);
    newMapThread.start();

    localMapWorker = QPointer<LocalMapWorker>(new LocalMapWorker(ip, PORT_LOCAL_MAP));
    connect(localMapWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(this, SIGNAL(stopLocalMapWorker()), localMapWorker, SLOT(stopWorker()));
    connect(this, SIGNAL(startLocalMapWorker()), localMapWorker, SLOT(connectSocket()));
    connect(&localMapThread, SIGNAL(finished()), localMapWorker, SLOT(deleteLater()));
    //qRegisterMetaType<QVector<float>>("QVector<float>");
    //connect(localMapWorker, SIGNAL(laserValues(float, float, float, QVector<float>, QString)), RobotController->getLaserController(), SLOT(drawObstacles(float,float,float,QVector<float>,QString)));
    localMapWorker->moveToThread(&localMapThread);
    localMapThread.start();

    mapWorker = QPointer<ScanMapWorker>(new ScanMapWorker(ip, PORT_MAP));
    connect(mapWorker, SIGNAL(valueChangedMap(QByteArray, int, QString, QString, QString, QString, QString, int, int)),
            this , SLOT(mapReceivedSlot(QByteArray, int, QString, QString, QString, QString, QString, int, int)));
    connect(mapWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(mapWorker, SIGNAL(newScanSaved(QString)), this , SLOT(sendNewMapToRobots(QString)));
    connect(&mapThread, SIGNAL(finished()), mapWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startMapWorker()), mapWorker, SLOT(connectSocket()));
    connect(this, SIGNAL(stopMapWorker()), mapWorker, SLOT(stopWorker()));
    mapWorker->moveToThread(&mapThread);
    mapThread.start();

    teleopWorker = QPointer<TeleopWorker>(new TeleopWorker(ip, PORT_TELEOP));
    connect(teleopWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(&mapThread, SIGNAL(finished()), teleopWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startTeleopWorker()), teleopWorker, SLOT(connectSocket()));
    connect(this, SIGNAL(stopTeleopWorker()), teleopWorker, SLOT(stopWorker()));
    connect(this, SIGNAL(teleopCmd(int)), teleopWorker, SLOT(writeTcpDataSlot(int)));
    teleopWorker->moveToThread(&mapThread);
    mapThread.start();

    particleCloudWorker = QPointer<ParticleCloudWorker>(new ParticleCloudWorker(ip, PORT_PARTICLE_CLOUD));
    connect(particleCloudWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(&particleCloudThread, SIGNAL(finished()), particleCloudWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startParticleCloudWorker()), particleCloudWorker, SLOT(connectSocket()));
    connect(this, SIGNAL(stopParticleCloudWorker()), particleCloudWorker, SLOT(stopWorker()));
    particleCloudWorker->moveToThread(&particleCloudThread);
    particleCloudThread.start();

    emit startCmdRobotWorker();
}

void RobotController::mapReceivedSlot(const QByteArray mapArray, int who, QString mapId, QString mapDate, QString resolution, QString originX, QString originY, int map_width, int map_height){
    qDebug() << "RobotController::mapReceivedSlot received a map" << who;

    switch(who){
        case 3:
            qDebug() << "RobotController::mapReceivedSlot received a map while scanning 3";
            /*QString robotName = robotsController->getRobots()->getRobotViewByIp(ipAddress)->getRobot()->getName();
            QImage image = mapController->getImageFromArray(mapArray, map_width, map_height, false);
            image.save(QDir::currentPath() + QDir::separator() + "brutos", "PNG");
            emit receivedScanMap(robotName, image, resolution.toDouble());*/
        break;
        case 2:
            qDebug() << "RobotController::mapReceivedSlot received a map from a robot to merge" << ip << resolution << originX << originY;
            qDebug() << "Map::getImageFromArray" << map_width << map_height << who;
            emit mapToMergeFromRobot(mapArray, resolution);
        break;
        case 1:
            emit newMapFromRobot(ip, mapArray, mapId, mapDate);
        break;
        case 0:
            qDebug() << "RobotController::mapReceivedSlot received a map while scanning 0";
            emit receivedScanMap(ip, mapArray, resolution);
            /*QString robotName = robotsController->getRobots()->getRobotViewByIp(ipAddress)->getRobot()->getName();
            QImage image = mapController->getImageFromArray(mapArray, false);
            image.save(QDir::currentPath() + QDir::separator() + "brutos", "PNG");
            emit receivedScanMap(robotName, image, mapController->getMap()->getResolution());*/
        break;
        default:
            Q_UNREACHABLE();
        break;
    }
    ping();
}

void RobotController::sendNewMapToRobots(QString){
    qDebug() << "RobotController::sendNewMapToRobots called";
}

void RobotController::doneSendingMapSlot(){
    qDebug() << "RobotController::doneSendingMapSlot called";
    sendingMap = false;
}

void RobotController::updateMetadata(int width, int height, float resolution, float originX, float originY){
    ping();
    emit newMetadata(width, height, resolution, originX, originY);
}

void RobotController::updateRobot(float posX, float posY, float ori){
    ping();
    emit newRobotPos(ip, posX, posY, ori);
}

void RobotController::robotIsDeadSlot(){
    qDebug() << "RobotController::robotIsDeadSlot called";
    emit robotIsDead(ip);
}

void RobotController::updateRobotInfo(QString robotInfo){

    QStringList strList = robotInfo.split(static_cast<u_char>(31), QString::SkipEmptyParts);
    qDebug() << "RobotController::updateRobotInfo" << strList;

    if(strList.size() > 7){
        /// Remove the "Connected" in the list
        strList.removeFirst();
        QString mapId = strList.takeFirst();
        QString mapDate = strList.takeFirst();
        QString homeName = strList.takeFirst();
        float homeX = static_cast<QString>(strList.takeFirst()).toFloat();
        float homeY = static_cast<QString>(strList.takeFirst()).toFloat();
        bool scanning = static_cast<QString>(strList.takeFirst()).toInt();
        bool recovering = static_cast<QString>(strList.takeFirst()).toInt();
        /// What remains in the list is the path

        emit updatePath(ip, strList);

        emit updateHome(ip, homeName, homeX, homeY);

        emit checkMapInfo(ip, mapId, mapDate);

        emit checkScanning(ip, scanning);
/*
        if(recovering){
            if(robotPositionRecoveryWidget){
                emit robotReconnected(robotName);
                playRecoverySlot(true, robotName);
            } else
                stopRecoveringRobotsSlot(QStringList(robotName));
        } else {
            if(robotPositionRecoveryWidget){
                QStringList robotRecoveringList = robotPositionRecoveryWidget->getAllRecoveringRobots();
                for(int i = 0; i < robotRecoveringList.count(); i++){
                    if(static_cast<QString>(robotRecoveringList.at(i)) == robotName){
                        emit robotReconnected(robotName);
                        emit robotRecovering(false, robotName, true);
                    }
                }
            }
        }
*/

    } else {
        qDebug() << "RobotController::updateRobotInfo Connected received without enough parameters :" << strList;
        Q_UNREACHABLE();
    }

    ping();
}

void RobotController::sendCommand(const QString cmd){
    qDebug() << "(RobotController) Send command called" << cmd;
    commandController->sendCommand(cmd);
}


void RobotController::sendNewMap(QString mapId, QString date, QString mapMetadata, QImage mapImage){
    qDebug() << "Robot::sendNewMap to" << ip;
    if(!sendingMap){
        sendingMap = true;
        emit sendNewMapSignal(mapId, date, mapMetadata, mapImage);
    } else
        qDebug() << "Robot::sendNewMap already sending a map to" << ip;
}

void RobotController::ping(void){
    emit pingSignal();
}

void RobotController::sendTeleop(int teleop){
    emit teleopCmd(teleop);
}
