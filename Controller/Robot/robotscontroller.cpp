#include "robotscontroller.h"
#include <QQmlProperty>
#include "Helper/helper.h"
#include "Controller/Robot/robotcontroller.h"
#include "Controller/Robot/robotserverworker.h"
#include "Controller/maincontroller.h"

RobotsController::RobotsController(QObject *applicationWindow, QQmlApplicationEngine* engine, MainController* parent) : QObject(parent), engine_(engine), robots(QMap<QString, QPointer<RobotController>>()), receivingMap(false) {

    QObject *robotModel = applicationWindow->findChild<QObject*>("robotModel");

    if (robotModel){
        /// Signals from the controller to the qml model
        connect(this, SIGNAL(displayRobots()), robotModel, SLOT(display()));
        connect(this, SIGNAL(addRobot(QVariant, QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(addRobot(QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(removeRobot(QVariant)),
                robotModel, SLOT(removeRobot(QVariant)));
        connect(this, SIGNAL(setPos(QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(setPos(QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(setHome(QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(setHome(QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(setName(QVariant, QVariant)),
                robotModel, SLOT(setName(QVariant, QVariant)));
        connect(this, SIGNAL(setPath(QVariant, QVariant)), robotModel, SLOT(setPath(QVariant, QVariant)));
        connect(this, SIGNAL(setPlayingPath(QVariant, QVariant)), robotModel, SLOT(setPlayingPath(QVariant, QVariant)));
        connect(this, SIGNAL(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(this, SIGNAL(setStage(QVariant, QVariant)), robotModel, SLOT(setStage(QVariant, QVariant)));
        connect(this, SIGNAL(setBattery(QVariant, QVariant)), robotModel, SLOT(setBattery(QVariant, QVariant)));
        connect(this, SIGNAL(setScanningOnConnection(QVariant, QVariant)), robotModel, SLOT(setScanningOnConnection(QVariant, QVariant)));
        connect(this, SIGNAL(processingCmd(QVariant, QVariant)), robotModel, SLOT(setProcessingCmd(QVariant, QVariant)));
        connect(this, SIGNAL(updateLaser(QVariant, QVariant)), robotModel, SLOT(setLaserActivated(QVariant, QVariant)));

        /// Signals from qml to the controller
        connect(robotModel, SIGNAL(newHomeSignal(QString, double, double, int)), parent, SLOT(sendCommandNewHome(QString, double, double, int)));
        connect(robotModel, SIGNAL(newPathSignal(QString, QString, QString)), parent, SLOT(sendCommandNewPath(QString, QString, QString)));
        connect(robotModel, SIGNAL(newNameSignal(QString, QString)), this, SLOT(sendCommandNewName(QString, QString)));
        connect(robotModel, SIGNAL(deletePathSignal(QString)), this, SLOT(sendCommandDeletePath(QString)));
        connect(robotModel, SIGNAL(pausePathSignal(QString)), this, SLOT(sendCommandPausePath(QString)));
        connect(robotModel, SIGNAL(playPathSignal(QString)), this, SLOT(sendCommandPlayPath(QString)));
        connect(robotModel, SIGNAL(stopPathSignal(QString)), this, SLOT(sendCommandStopPath(QString)));
        connect(robotModel, SIGNAL(stopScanning(QString, bool)), parent, SLOT(stopScanningSlot(QString, bool)));
        connect(robotModel, SIGNAL(activateLaser(QString, bool)), this, SLOT(activateLaserSlot(QString, bool)));


        /// MainController signals
        connect(parent, SIGNAL(setPath(QVariant, QVariant)), robotModel, SLOT(setPath(QVariant, QVariant)));
        connect(parent, SIGNAL(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(parent, SIGNAL(setHome(QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(setHome(QVariant, QVariant, QVariant, QVariant)));

        connect(this, SIGNAL(receivedScanMap(QString, QByteArray, QString, QString, QString, int, int)),
                parent, SLOT(receivedScanMapSlot(QString, QByteArray, QString, QString, QString, int, int)));


    } else {
        /// NOTE can probably remove that when testing phase is over
        qDebug() << "RobotsController::RobotsController could not find the qml robot model";
        Q_UNREACHABLE();
    }


    QObject* scanLeftMenuFrame = applicationWindow->findChild<QObject*>("scanLeftMenuFrame");

    if(scanLeftMenuFrame){
        connect(this, SIGNAL(stoppedScanning(QVariant)), scanLeftMenuFrame, SLOT(stoppedScanning(QVariant)));
        connect(this, SIGNAL(startedScanning(QVariant)), scanLeftMenuFrame, SLOT(startedScanning(QVariant)));
        connect(this, SIGNAL(pausedScanning(QVariant)), scanLeftMenuFrame, SLOT(pausedScanning(QVariant)));
        connect(this, SIGNAL(checkScanWindow(QVariant, QVariant)), scanLeftMenuFrame, SLOT(checkScanWindow(QVariant, QVariant)));
    }

    connect(this, SIGNAL(newRobotPos(QString, double, double, double)), parent, SLOT(newRobotPosSlot(QString, double, double, double)));
    connect(this, SIGNAL(updatePath(QString, QStringList)), parent, SLOT(updatePathSlot(QString, QStringList)));
    connect(this, SIGNAL(updateHome(QString, double, double, double)), parent, SLOT(updateHomeSlot(QString, double, double, double)));
    connect(this, SIGNAL(checkMapInfo(QString, QString, QString)), parent, SLOT(checkMapInfoSlot(QString, QString, QString)));
    connect(this, SIGNAL(newMapFromRobot(QString, QByteArray, QString, QString, QString, QString, QString, int, int)),
            parent, SLOT(newMapFromRobotSlot(QString, QByteArray, QString, QString, QString, QString, QString, int, int)));

    connect(this, SIGNAL(sendMapToProcessForMerge(QByteArray, QString)), parent, SLOT(processMapForMerge(QByteArray, QString)));
    connect(this, SIGNAL(removeScanMap(QString)), parent, SLOT(removeScanMapSlot(QString)));
    connect(this, SIGNAL(setMessageTop(int, QString)), parent, SLOT(setMessageTopSlot(int, QString)));

    QObject* robotMenuFrame = applicationWindow->findChild<QObject*>("robotMenuFrame");
    if(robotMenuFrame){
        connect(robotMenuFrame, SIGNAL(dockRobot(QString)), this, SLOT(dockRobot(QString)));
    } else {
        qDebug() << "could not find robot menu frame";
        Q_UNREACHABLE();
    }

    launchServer();
}

RobotsController::~RobotsController(){
    if(robotServerWorker){
        emit stopRobotServerWorker();
        serverThread.quit();
        serverThread.wait();
    }
}

void RobotsController::launchServer(void){
    robotServerWorker = QPointer<RobotServerWorker>(new RobotServerWorker(PORT_ROBOT_UPDATE));
    connect(robotServerWorker, SIGNAL(robotIsAlive(QString, QString, QString, int, int)), this, SLOT(robotIsAliveSlot(QString, QString, QString, int, int)));
    connect(this, SIGNAL(stopRobotServerWorker()), robotServerWorker, SLOT(stopWorker()));
    connect(&serverThread, SIGNAL(finished()), robotServerWorker, SLOT(deleteLater()));
    serverThread.start();
    robotServerWorker->moveToThread(&serverThread);
}

void RobotsController::robotIsAliveSlot(const QString name, const QString ip, const QString ssid, const int stage, const int battery){
    //qDebug() << "RobotsController::robotIsAliveSlot" << name << ip << ssid << stage << battery;
    if(robots.find(ip) != robots.end()){
        emit setStage(ip, stage);
        emit setBattery(ip, battery);
        robots.value(ip)->ping();
    } else {
        QPointer<RobotController> robotController = QPointer<RobotController>(new RobotController(engine_, this, ip, name));

        robots.insert(ip, robotController);
        emit addRobot(name, ip, ssid, stage, battery);
    }
}

void RobotsController::robotIsDeadSlot(const QString ip){
    if(robots.contains(ip)){
        robots.take(ip)->deleteLater();
        emit removeRobot(ip);
    }
}

void RobotsController::shortcutAddRobot(void){
    QString ip = QString::number(robots.size());
    double posX = ((robots.size() + 1) * 200) % 1555;
    double posY = ((robots.size() + 1) * 200) % 1222;

    robotIsAliveSlot("Robot avec un nom tres tres long " + ip, ip, "Wifi " + ip, 0, (robots.size()*10)%100);
    emit setPos(ip, posX, posY, (20 * robots.size()) % 360);

    if((robots.size() - 1)%2 == 0){
        emit setHome(ip, posX + 50, posY, static_cast<int>(posX) % 360);
        emit setPlayingPath(ip, (robots.size() - 1)%2 == 0);
    }

    /// NOTE remove when tests ok
    if((robots.size() - 1)%3 == 0){
        emit setPath(ip, "pathName avec un nom tres tres long " + ip);
        for(int i = 1; i <= robots.size(); i++)
            emit addPathPoint(ip, "pathPoint " + QString::number(i), 50 * robots.size() + 50*i%3, 50 * robots.size() + 50*i%3, (robots.size() - 1)%3);
        emit setStage(ip, (int) ((robots.size() - 1) / 3));
    }
}

void RobotsController::shortcutDeleteRobot(void){
    if(robots.size() > 0)
        robotIsDeadSlot(QString::number(robots.size() - 1));
    else
        qDebug() << "You already have no robot";
}

void RobotsController::sendCommand(const QString ip, const QString cmd){
    if(robots.contains(ip))
        robots.value(ip)->sendCommand(cmd);
    else
        qDebug() << "RobotsController::sendCommand Trying to send a command to a robot which is disconnected";
}

void RobotsController::newRobotPosSlot(const QString ip, const double posX, const double posY, const double ori){
    emit newRobotPos(ip, posX, posY, ori);
}

void RobotsController::setRobotPos(const QString ip, const double posX, const double posY, const double ori){
    emit setPos(ip, posX, posY, ori);
}

void RobotsController::updatePathSlot(const QString ip, const QStringList strList){
    emit updatePath(ip, strList);
}

void RobotsController::updateHomeSlot(const QString ip, const double homeX, const double homeY, const double homeOri){
    emit updateHome(ip, homeX, homeY, homeOri);
}

void RobotsController::sendCommandNewName(const QString ip, const QString name){
    sendCommand(ip, QString("a") + QChar(31) + name);
}

void RobotsController::updateNameSlot(const QString ip, const QString name){
    emit setName(ip, name);
}

void RobotsController::sendCommandDeletePath(const QString ip){
    sendCommand(ip, QString("m"));
}

void RobotsController::stoppedDeletedPathSlot(const QString ip){
    emit setPath(ip, "");
    emit setPlayingPath(ip, false);
}

void RobotsController::sendCommandPausePath(const QString ip){
    sendCommand(ip, QString("d"));
}

void RobotsController::sendCommandPlayPath(const QString ip){
    sendCommand(ip, QString("j"));
}

void RobotsController::sendCommandStopPath(const QString ip){
    sendCommand(ip, QString("l"));
}

void RobotsController::updatePlayingPathSlot(const QString ip, const bool playingPath){
    emit setPlayingPath(ip, playingPath);
}

void RobotsController::checkMapInfoSlot(const QString ip, const QString mapId, const QString mapDate){
    emit checkMapInfo(ip, mapId, mapDate);
}

void RobotsController::sendNewMap(const QString ip, const QString mapId, const QString date, const QString mapMetadata, const QImage mapImage) {
    robots.value(ip)->sendNewMap(mapId, date, mapMetadata, mapImage);
}

void RobotsController::newMapFromRobotSlot(const QString ip, const QByteArray mapArray, const QString mapId, const QString mapDate, const QString resolution, const QString originX, const QString originY, const int map_width, const int map_height){
    emit newMapFromRobot(ip, mapArray, mapId, mapDate, resolution, originX, originY, map_width, map_height);
}

void RobotsController::requestMap(const QString ip){
    qDebug() << "Requesting the map from robot at ip" << ip;
    if(!receivingMap){
        sendCommand(ip, QString("s") + QChar(31) + QString::number(1));
        receivingMap = true;
    }
}

void RobotsController::sendNewMapToAllExcept(const QString ip, const QString mapId, const QString date, const QString mapMetadata, const QImage mapImage) {
    QList<QString> ipList = robots.keys();
    for(int i = 0; i < ipList.size(); i++)
        if(ipList.at(i).compare(ip) != 0)
            sendNewMap(ipList.at(i), mapId, date, mapMetadata, mapImage);

    timer = new QTimer(this);
    timer->setInterval(10000);

    connect(timer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    timer->start();
}

void RobotsController::timerSlot(void){
    qDebug() << "RobotsController::timerSlot should have sent all the map already";
    receivingMap = false;
    timer->stop();
}

void RobotsController::requestMapForMerging(const QString ip){
    if(!receivingMap){
        sendCommand(ip, QString("s") + QChar(31) + QString::number(2));
        receivingMap = true;
    }
}

void RobotsController::processMapForMerge(const QByteArray map, const QString resolution){
    qDebug() << "RobotsController::processMapForMerge";
    emit sendMapToProcessForMerge(map, resolution);
}

void RobotsController::startedScanningSlot(const QString ip){
    emit startedScanning(ip);
}

void RobotsController::stoppedScanningSlot(const QString ip){
    emit stoppedScanning(ip);
    emit removeScanMap(ip);
}

void RobotsController::pausedScanningSlot(const QString ip){
    emit pausedScanning(ip);
}

void RobotsController::receivedScanMapSlot(const QString ip, const QByteArray map, const QString resolution, const QString originX, const QString originY, const int map_width, const int map_height){
    emit receivedScanMap(ip, map, resolution, originX, originY, map_width, map_height);
}

void RobotsController::sendTeleop(const QString ip, const int teleop){
    if(robots.contains(ip))
        robots.value(ip)->sendTeleop(teleop);
    else
        qDebug() << "RobotsController::sendTeleop Trying to send a teleop cmd to a robot which is disconnected";
}

void RobotsController::sendMapToAllRobots(QString mapId, QString date, QString mapMetadata, QImage img){
    QMapIterator<QString, QPointer<RobotController>> it(robots);
    while(it.hasNext()){
        it.next();
        robots.value(it.key())->sendNewMap(mapId, date, mapMetadata, img);
    }
}

void RobotsController::checkScanningSlot(const QString ip, const bool scanning){
    /// update the robot model
    emit setScanningOnConnection(ip, scanning);

    /// update the scanning menu
    if(scanning)
        emit startedScanning(ip);
    else
        emit pausedScanning(ip);

    /// Stop the scan if a scanning robot reconnect after the window has been closed
    emit checkScanWindow(ip, scanning);
}

void RobotsController::processingCmdSlot(QString ip, bool processing){
    emit processingCmd(ip, processing);
}

void RobotsController::setMessageTopSlot(int status, QString msg){
    emit setMessageTop(status, msg);
}

void RobotsController::activateLaserSlot(QString ip, bool activate){
    if(activate)
        sendCommand(ip, QString("q"));
    else
        sendCommand(ip, QString("r"));
}

void RobotsController::updateLaserSlot(QString ip, bool activated){
    if(robots.contains(ip))
        robots.value(ip)->clearObstacles(activated);

    emit updateLaser(ip, activated);
}

void RobotsController::updateRobotPos(QString ip, double x, double y, double orientation){
    robots.value(ip)->updateRobotPosition(x, y, orientation);
}

void RobotsController::dockRobot(QString ip){
    sendCommand(ip, QString("0"));
}

void RobotsController::resetHomePathSlot(QString ip){
    emit setHome(ip, 0, 0, 0);
    emit setPath(ip, "");
}

