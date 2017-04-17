#include "robotscontroller.h"
#include <QQmlProperty>
#include "Helper/helper.h"
#include "Controller/Robot/robotcontroller.h"
#include "Controller/Robot/robotserverworker.h"
#include "Controller/maincontroller.h"

RobotsController::RobotsController(QObject *applicationWindow, MainController* parent) : QObject(parent), robots(QMap<QString, QPointer<RobotController>>()), receivingMap(false){

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

        /// Signals from qml to the controller
        connect(robotModel, SIGNAL(newHomeSignal(QString, QString, double, double)), parent, SLOT(sendCommandNewHome(QString, QString, double, double)));
        connect(robotModel, SIGNAL(newPathSignal(QString, QString, QString)), parent, SLOT(sendCommandNewPath(QString, QString, QString)));
        connect(robotModel, SIGNAL(newNameSignal(QString, QString)), this, SLOT(sendCommandNewName(QString, QString)));
        connect(robotModel, SIGNAL(deletePathSignal(QString)), this, SLOT(sendCommandDeletePath(QString)));
        connect(robotModel, SIGNAL(pausePathSignal(QString)), this, SLOT(sendCommandPausePath(QString)));
        connect(robotModel, SIGNAL(playPathSignal(QString)), this, SLOT(sendCommandPlayPath(QString)));
        connect(robotModel, SIGNAL(stopPathSignal(QString)), this, SLOT(sendCommandStopPath(QString)));
        connect(robotModel, SIGNAL(stopScanning(QString)), parent, SLOT(stopScanningSlot(QString)));


        /// MainController signals
        connect(parent, SIGNAL(setPath(QVariant, QVariant)), robotModel, SLOT(setPath(QVariant, QVariant)));
        connect(parent, SIGNAL(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(parent, SIGNAL(setHome(QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(setHome(QVariant, QVariant, QVariant, QVariant)));

        connect(this, SIGNAL(receivedScanMap(QString, QByteArray, QString)),
                parent, SLOT(receivedScanMapSlot(QString, QByteArray, QString)));
    } else {
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

    connect(this, SIGNAL(newRobotPos(QString, float, float, float)), parent, SLOT(newRobotPosSlot(QString, float, float, float)));
    connect(this, SIGNAL(newMetadata(int, int, float, float, float)), parent, SLOT(newMetadataSlot(int, int, float, float, float)));
    connect(this, SIGNAL(updatePath(QString, QStringList)), parent, SLOT(updatePathSlot(QString, QStringList)));
    connect(this, SIGNAL(updateHome(QString, QString, float, float)), parent, SLOT(updateHomeSlot(QString, QString, float, float)));
    connect(this, SIGNAL(checkMapInfo(QString, QString, QString)), parent, SLOT(checkMapInfoSlot(QString, QString, QString)));
    connect(this, SIGNAL(newMapFromRobot(QString, QByteArray, QString, QString)), parent, SLOT(newMapFromRobotSlot(QString, QByteArray, QString, QString)));
    connect(this, SIGNAL(sendMapToProcessForMerge(QByteArray, QString)), parent, SLOT(processMapForMerge(QByteArray, QString)));
    connect(this, SIGNAL(removeScanMap(QString)), parent, SLOT(removeScanMapSlot(QString)));

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
        QPointer<RobotController> robotController = QPointer<RobotController>(new RobotController(this, ip));
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
        emit setHome(ip, "home avec un nom tres tres long " + ip, posX + 50, posY);
        emit setPlayingPath(ip, (robots.size() - 1)%2 == 0);
    }

    /// TODO remove when tests ok
    if((robots.size() - 1)%3 == 0){
        emit setPath(ip, "pathName avec un nom tres tres long " + ip);
        emit addPathPoint(ip, QString("pathPoint 1"), 50 * robots.size() + 50, 50 * robots.size() + 50, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 2"), 50 * robots.size() + 50*2, 50 * robots.size() + 50*2, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 3"), 50 * robots.size() + 50*3, 50 * robots.size() + 50*3, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 4"), 50 * robots.size() + 50*4, 50 * robots.size() + 50*4, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 5"), 50 * robots.size() + 50*5, 50 * robots.size() + 50*5, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 6"), 50 * robots.size() + 50*6, 50 * robots.size() + 50*6, (robots.size() - 1)%3);
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

void RobotsController::newRobotPosSlot(const QString ip, const float posX, const float posY, const float ori){
    emit newRobotPos(ip, posX, posY, ori);
}

void RobotsController::setRobotPos(const QString ip, const float posX, const float posY, const float ori){
    emit setPos(ip, posX, posY, ori);
}

void RobotsController::newMetadataSlot(const int width, const int height, const float resolution, const float originX, const float originY){
    emit newMetadata(width, height, resolution, originX, originY);
}

void RobotsController::updatePathSlot(const QString ip, const QStringList strList){
    emit updatePath(ip, strList);
}

void RobotsController::updateHomeSlot(const QString ip, const QString homeName, const float homeX, const float homeY){
    emit updateHome(ip, homeName, homeX, homeY);
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
    emit setHome(ip, "", 0, 0);
    emit setPath(ip, "");
}

void RobotsController::newMapFromRobotSlot(const QString ip, const QByteArray mapArray, const QString mapId, const QString mapDate){
    emit newMapFromRobot(ip, mapArray, mapId, mapDate);
}

void RobotsController::requestMap(const QString ip){
    if(!receivingMap){
        sendCommand(ip, QString("s") + QChar(31) + QString::number(1));
        receivingMap = true;
    }
}

void RobotsController::sendNewMapToAllExcept(const QString ip, const QString mapId, const QString date, const QString mapMetadata, const QImage mapImage) {
    QList<QString> ipList = robots.keys();
    for(int i = 0; i < ipList.size(); i++)
        if(ipList.at(i).compare(ip) != 0)
            sendNewMap(ip, mapId, date, mapMetadata, mapImage);

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

void RobotsController::receivedScanMapSlot(const QString ip, const QByteArray map, const QString resolution){
    emit receivedScanMap(ip, map, resolution);
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
