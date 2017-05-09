#include "robotcontroller.h"
#include <QApplication>
#include <QDir>
#include <QDebug>
#include <QtMath>
#include <QQmlApplicationEngine>
#include "Helper/helper.h"
#include "Controller/Robot/robotscontroller.h"
#include "Controller/Robot/cmdrobotworker.h"
#include "Controller/Robot/robotpositionworker.h"
#include "Controller/Robot/teleopworker.h"
#include "Controller/Robot/commandcontroller.h"
#include "Controller/Map/sendnewmapworker.h"
#include "Controller/Map/localmapworker.h"
#include "Controller/Map/scanmapworker.h"
#include "Controller/Map/particlecloudworker.h"
#include "Controller/maincontroller.h"
#include "Controller/Map/mapcontroller.h"
#include "View/Robot/obstaclespainteditem.h"

#define PI 3.14159265

RobotController::RobotController(QQmlApplicationEngine* engine, RobotsController *parent, QString _ip, QString robotName):
    QObject(parent), ip(_ip), sendingMap(false), commandController(QPointer<CommandController>(new CommandController(this, ip, robotName))){

    /// Signals from the command controller when we have executed a command
    connect(commandController, SIGNAL(updateName(QString, QString)), parent, SLOT(updateNameSlot(QString, QString)));
    connect(commandController, SIGNAL(updateHome(QString, double, double, double)), parent, SLOT(updateHomeSlot(QString, double, double, double)));
    connect(commandController, SIGNAL(updatePath(QString, QStringList)), parent, SLOT(updatePathSlot(QString, QStringList)));
    connect(commandController, SIGNAL(stoppedDeletedPath(QString)), parent, SLOT(stoppedDeletedPathSlot(QString)));
    connect(commandController, SIGNAL(updatePlayingPath(QString, bool)), parent, SLOT(updatePlayingPathSlot(QString, bool)));
    connect(commandController, SIGNAL(startedScanning(QString)), parent, SLOT(startedScanningSlot(QString)));
    connect(commandController, SIGNAL(stoppedScanning(QString)), parent, SLOT(stoppedScanningSlot(QString)));
    connect(commandController, SIGNAL(playedScanning(QString)), parent, SLOT(startedScanningSlot(QString)));
    connect(commandController, SIGNAL(pausedScanning(QString)), parent, SLOT(pausedScanningSlot(QString)));
    connect(commandController, SIGNAL(processingCmd(QString, bool)), parent, SLOT(processingCmdSlot(QString, bool)));
    connect(commandController, SIGNAL(setMessageTop(int, QString)), parent, SLOT(setMessageTopSlot(int, QString)));
    connect(commandController, SIGNAL(updateLaser(QString, bool)), parent, SLOT(updateLaserSlot(QString, bool)));

    /// Signals to tell the robotsController that the robot just disconnected
    connect(this, SIGNAL(robotIsDead(QString)), parent, SLOT(robotIsDeadSlot(QString)));
    /// Update the position of the robot
    connect(this, SIGNAL(newRobotPos(QString, double, double, double)), parent, SLOT(newRobotPosSlot(QString, double, double, double)));
    /// Update the path of the robot when it connects
    connect(this, SIGNAL(updatePath(QString, QStringList)), parent, SLOT(updatePathSlot(QString, QStringList)));
    /// Update the home of the robot when it connects
    connect(this, SIGNAL(updateHome(QString, double, double, double)), parent, SLOT(updateHomeSlot(QString, double, double, double)));
    /// Check if the robot has the same map as the application
    connect(this, SIGNAL(checkMapInfo(QString, QString, QString)), parent, SLOT(checkMapInfoSlot(QString, QString, QString)));
    /// Signal that we just received a new map fron the robot
    connect(this, SIGNAL(newMapFromRobot(QString, QByteArray, QString, QString, QString, QString, QString, int, int)),
            parent, SLOT(newMapFromRobotSlot(QString, QByteArray, QString, QString, QString, QString, QString, int, int)));
    /// Signal that we just received a map to merge from the robot
    connect(this, SIGNAL(mapToMergeFromRobot(QByteArray, QString)), parent, SLOT(processMapForMerge(QByteArray, QString)));
    /// Signal that we received a new map while scanning
    connect(this, SIGNAL(receivedScanMap(QString, QByteArray, QString, QString, QString, int, int)),
            parent, SLOT(receivedScanMapSlot(QString, QByteArray, QString, QString, QString, int, int)));
    /// Check if the robot is scanning when it connects
    connect(this, SIGNAL(checkScanning(QString, bool)), parent, SLOT(checkScanningSlot(QString, bool)));
    connect(this, SIGNAL(updateLaser(QString, bool)), parent, SLOT(updateLaserSlot(QString, bool)));

    /// to draw the obstacles of the robots
    QQmlComponent component(engine, QUrl("qrc:/View/Robot/ObstaclesItems.qml"));
    paintedItem = qobject_cast<ObstaclesPaintedItem*>(component.create());
    QQmlEngine::setObjectOwnership(paintedItem, QQmlEngine::CppOwnership);

    QList<QObject*> qmlList = engine->rootObjects();
    /// The main parent element in the QML tree
    QObject *applicationWindow = qmlList.at(0);

    QQuickItem* mapView = applicationWindow->findChild<QQuickItem*> ("mapImage");
    paintedItem->setParentItem(mapView);
    paintedItem->setParent(this);

    launchWorkers();
}

RobotController::~RobotController(){
    stopThreads();
}

void RobotController::stopThreads(void) {

    emit stopCmdRobotWorker();
    cmdThread.quit();
    cmdThread.wait();

    emit stopRobotWorker();
    robotThread.quit();
    robotThread.wait();

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

    emit stopParticleCloudWorker();
    particleCloudThread.quit();
    particleCloudThread.wait();

    qDebug() << "RobotController::stopThreads" << ip << "all threads stopped";
}

void RobotController::portSentSlot(void){
    emit startRobotWorker();
    emit startNewMapWorker();
    emit startLocalMapWorker();
    emit startMapWorker();
    emit startTeleopWorker();
    emit startParticleCloudWorker();
}


void RobotController::launchWorkers(void){

    if(ip.length() < 3)
        return;

    qDebug() << "RobotController at ip" << ip << " launching its cmd thread";

    cmdRobotWorker = QPointer<CmdRobotWorker>(new CmdRobotWorker(ip, PORT_CMD, PORT_ROBOT_POS, PORT_MAP, PORT_LOCAL_MAP));
    connect(cmdRobotWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(cmdRobotWorker, SIGNAL(cmdAnswer(QString)), commandController, SLOT(cmdAnswerSlot(QString)));
    connect(cmdRobotWorker, SIGNAL(portSent()), this, SLOT(portSentSlot()));
    connect(cmdRobotWorker, SIGNAL(newConnection(QString)), this, SLOT(updateRobotInfo(QString)));
    connect(commandController, SIGNAL(sendCommandSignal(QString)), cmdRobotWorker, SLOT(sendCommand(QString)));
    connect(this, SIGNAL(pingSignal()), cmdRobotWorker, SLOT(pingSlot()));
    connect(this, SIGNAL(stopCmdRobotWorker()), cmdRobotWorker, SLOT(stopWorker()));
    connect(&cmdThread, SIGNAL(finished()), cmdRobotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startCmdRobotWorker()), cmdRobotWorker, SLOT(connectSocket()));
    cmdRobotWorker->moveToThread(&cmdThread);
    cmdThread.start();


    robotWorker = QPointer<RobotPositionWorker>(new RobotPositionWorker(ip, PORT_ROBOT_POS));
    connect(robotWorker, SIGNAL(valueChangedRobot(double, double, double)),
                     this ,SLOT(updateRobot(double, double, double)));
    connect(robotWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
    connect(this, SIGNAL(stopRobotWorker()), robotWorker, SLOT(stopWorker()));
    connect(&robotThread, SIGNAL(finished()), robotWorker, SLOT(deleteLater()));
    connect(this, SIGNAL(startRobotWorker()), robotWorker, SLOT(connectSocket()));
    robotWorker->moveToThread(&robotThread);
    robotThread.start();


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
    qRegisterMetaType<QVector<double>>("QVector<double>");
    connect(localMapWorker, SIGNAL(laserValues(double, double, double, QVector<double>)),
            this, SLOT(updateObstacles(double, double, double, QVector<double>)));
    localMapWorker->moveToThread(&localMapThread);
    localMapThread.start();


    mapWorker = QPointer<ScanMapWorker>(new ScanMapWorker(ip, PORT_MAP));
    connect(mapWorker, SIGNAL(valueChangedMap(QByteArray, int, QString, QString, QString, QString, QString, int, int)),
            this , SLOT(mapReceivedSlot(QByteArray, int, QString, QString, QString, QString, QString, int, int)));
    connect(mapWorker, SIGNAL(robotIsDead()), this, SLOT(robotIsDeadSlot()));
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

void RobotController::mapReceivedSlot(const QByteArray mapArray, const int who, const QString mapId, const QString mapDate, const QString resolution, const QString originX, const QString originY, const int map_width, const int map_height){
    qDebug() << "RobotController::mapReceivedSlot received a map" << who;

    switch(who){
        case 3:
            qDebug() << "RobotController::mapReceivedSlot received a map while recovering";
            /*QString robotName = robotsController->getRobots()->getRobotViewByIp(ipAddress)->getRobot()->getName();
            QImage image = mapController->getImageFromArray(mapArray, map_width, map_height, false);
            image.save(Helper::getAppPath() + QDir::separator() + "brutos", "PNG");
            emit receivedScanMap(robotName, image, resolution.toDouble());*/
        break;
        case 2:
            qDebug() << "RobotController::mapReceivedSlot received a map from a robot to merge" << ip << resolution << originX << originY;
            emit mapToMergeFromRobot(mapArray, resolution);
        break;
        case 1:
            emit newMapFromRobot(ip, mapArray, mapId, mapDate, resolution, originX, originY, map_width, map_height);
        break;
        case 0:
            qDebug() << "RobotController::mapReceivedSlot received a map while scanning";
            emit receivedScanMap(ip, mapArray, resolution, originX, originY, map_width, map_height);
        break;
        default:
        /// NOTE can probably remove that when testing phase is over
            Q_UNREACHABLE();
        break;
    }
    ping();
}

void RobotController::doneSendingMapSlot(void){
    qDebug() << "RobotController::doneSendingMapSlot called";
    sendingMap = false;
}

void RobotController::updateRobot(const double posX, const double posY, const double ori){
    ping();
    emit newRobotPos(ip, posX, posY, ori);
}

void RobotController::robotIsDeadSlot(void){
    qDebug() << "RobotController::robotIsDeadSlot called";
    emit robotIsDead(ip);
}

void RobotController::updateRobotInfo(const QString robotInfo){

    QStringList strList = robotInfo.split(QChar(31), QString::SkipEmptyParts);
    qDebug() << "RobotController::updateRobotInfo" << strList;

    if(strList.size() > 7){
        /// Remove the "Connected" in the list
        strList.removeFirst();
        QString mapId = strList.takeFirst();
        QString mapDate = strList.takeFirst();
        double homeX = static_cast<QString>(strList.takeFirst()).toDouble();
        double homeY = static_cast<QString>(strList.takeFirst()).toDouble();
        double homeOri = static_cast<QString>(strList.takeFirst()).toDouble();
        bool scanning = static_cast<QString>(strList.takeFirst()).toInt();
        bool recovering = static_cast<QString>(strList.takeFirst()).toInt();
        bool laser = static_cast<QString>(strList.takeFirst()).toInt();
        /// What remains in the list is the path

        if(!strList.empty())
            emit updatePath(ip, strList);

        if(homeX != -1 && homeY != -1)
            emit updateHome(ip, homeX, homeY, homeOri);

        emit checkMapInfo(ip, mapId, mapDate);

        emit checkScanning(ip, scanning);

        emit updateLaser(ip, laser);
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
        /// NOTE what to do if something is missing ? should not happen as the user should not be able to access the robot files
        qDebug() << "RobotController::updateRobotInfo Connected received without enough parameters :" << strList;
        //Q_UNREACHABLE();
    }

    ping();
}

void RobotController::sendCommand(const QString cmd){
    qDebug() << "(RobotController) Send command called" << cmd;
    commandController->sendCommand(cmd);
}


void RobotController::sendNewMap(const QString mapId, const QString date, const QString mapMetadata, const QImage &mapImage){
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

void RobotController::updateObstacles(double angle_min, double angle_max, double angle_increment, QVector<double> ranges){
    paintedItem->updateObstacles(angle_min, angle_max, angle_increment, ranges);
}

void RobotController::clearObstacles(bool activated){
    paintedItem->clearObstacles(activated);
}

void RobotController::updateRobotPosition(double x, double y, double orientation){
    paintedItem->setProperty("orientation_", orientation);
    paintedItem->setProperty("_x", x-300 + 5 * qCos((paintedItem->orientation() - 90) / 180.0*3.14159));
    paintedItem->setProperty("_y", y-300 + 5 * qSin((paintedItem->orientation() - 90) / 180.0*3.14159));
}
