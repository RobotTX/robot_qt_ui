#include "robotscontroller.h"
#include "Helper/helper.h"
#include "Controller/Robot/robotcontroller.h"
#include "Controller/Robot/robotserverworker.h"
#include "Controller/maincontroller.h"

RobotsController::RobotsController(QObject *applicationWindow, MainController* parent) : QObject(parent), robots(QMap<QString, QPointer<RobotController>>()){

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

        /// Signals from qml to the controller
        connect(robotModel, SIGNAL(newHomeSignal(QString, QString, double, double)), parent, SLOT(sendCommandNewHome(QString, QString, double, double)));
        connect(robotModel, SIGNAL(newPathSignal(QString, QString, QString)), parent, SLOT(sendCommandNewPath(QString, QString, QString)));
        connect(robotModel, SIGNAL(newNameSignal(QString, QString)), this, SLOT(sendCommandNewName(QString, QString)));
        connect(robotModel, SIGNAL(deletePathSignal(QString)), this, SLOT(sendCommandDeletePath(QString)));


        /// MainController signals
        connect(parent, SIGNAL(setPath(QVariant, QVariant)), robotModel, SLOT(setPath(QVariant, QVariant)));
        connect(parent, SIGNAL(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(addPathPoint(QVariant, QVariant, QVariant, QVariant, QVariant)));
        connect(parent, SIGNAL(setHome(QVariant, QVariant, QVariant, QVariant)),
                robotModel, SLOT(setHome(QVariant, QVariant, QVariant, QVariant)));
    } else {
        qDebug() << "RobotsController::RobotsController could not find the qml robot model";
        Q_UNREACHABLE();
    }


    connect(this, SIGNAL(newRobotPos(QString, float, float, float)), parent, SLOT(newRobotPosSlot(QString, float, float, float)));
    connect(this, SIGNAL(newMetadata(int, int, float, float, float)), parent, SLOT(newMetadataSlot(int, int, float, float, float)));
    connect(this, SIGNAL(updatePath(QString, QStringList)), parent, SLOT(updatePathSlot(QString, QStringList)));
    connect(this, SIGNAL(updateHome(QString, QString, float, float)), parent, SLOT(updateHomeSlot(QString, QString, float, float)));

    launchServer();
}

RobotsController::~RobotsController(){
    if(robotServerWorker){
        emit stopRobotServerWorker();
        serverThread.quit();
        serverThread.wait();
    }
/*
    QMapIterator<QString, QPointer<RobotController>> i(robots);
    while (i.hasNext()) {
        i.next();
        i.value()->stopThreads();
    }*/
}

void RobotsController::launchServer(){
    robotServerWorker = new RobotServerWorker(PORT_ROBOT_UPDATE);
    connect(robotServerWorker, SIGNAL(robotIsAlive(QString, QString, QString, int, int)), this, SLOT(robotIsAliveSlot(QString, QString, QString, int, int)));
    connect(this, SIGNAL(stopRobotServerWorker()), robotServerWorker, SLOT(stopWorker()));
    connect(&serverThread, SIGNAL(finished()), robotServerWorker, SLOT(deleteLater()));
    serverThread.start();
    robotServerWorker->moveToThread(&serverThread);
}

void RobotsController::robotIsAliveSlot(QString name, QString ip, QString ssid, int stage, int battery){
    //qDebug() << "RobotsController::robotIsAliveSlot" << name << ip << ssid << stage << battery;
    if(robots.find(ip) != robots.end()){
        /// TODO update battery + stage + ping RobotController
    } else {
        QPointer<RobotController> robotController = QPointer<RobotController>(new RobotController(this, ip));
        robots.insert(ip, robotController);
        connect(robotController, SIGNAL(robotIsDead(QString)), this, SLOT(robotIsDeadSlot(QString)));
        connect(robotController, SIGNAL(newRobotPos(QString, float, float, float)), this, SLOT(newRobotPosSlot(QString, float, float, float)));
        connect(robotController, SIGNAL(newMetadata(int, int, float, float, float)), this, SLOT(newMetadataSlot(int, int, float, float, float)));
        connect(robotController, SIGNAL(updatePath(QString, QStringList)), this, SLOT(updatePathSlot(QString, QStringList)));
        connect(robotController, SIGNAL(updateHome(QString, QString, float, float)), this, SLOT(updateHomeSlot(QString, QString, float, float)));
        emit addRobot(name, ip, ssid, stage, battery);
    }
}

void RobotsController::robotIsDeadSlot(QString ip){
    robots.remove(ip);
    emit removeRobot(ip);
}

void RobotsController::shortcutAddRobot(){
    QString ip = QString::number(robots.size());
    double posX = ((robots.size() + 1) * 200) % 1555;
    double posY = ((robots.size() + 1) * 200) % 1222;

    robotIsAliveSlot("Robot " + ip, ip, "Wifi " + ip, 0, (robots.size()*10)%100);
    emit setPos(ip, posX, posY, (20 * robots.size()) % 360);

    if((robots.size() - 1)%2 == 0){
        emit setHome(ip, "home " + ip, posX + 50, posY);
        emit setPlayingPath(ip, (robots.size() - 1)%4/2);
    }

    if((robots.size() - 1)%3 == 0){
        emit setPath(ip, "pathName " + ip);
        emit addPathPoint(ip, QString("pathPoint 1"), 50 * robots.size() + 50, 50 * robots.size() + 50, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 2"), 50 * robots.size() + 50*2, 50 * robots.size() + 50*2, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 3"), 50 * robots.size() + 50*3, 50 * robots.size() + 50*3, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 4"), 50 * robots.size() + 50*4, 50 * robots.size() + 50*4, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 5"), 50 * robots.size() + 50*5, 50 * robots.size() + 50*5, (robots.size() - 1)%3);
        emit addPathPoint(ip, QString("pathPoint 6"), 50 * robots.size() + 50*6, 50 * robots.size() + 50*6, (robots.size() - 1)%3);
        emit setStage(ip, (int) ((robots.size() - 1) / 3));
    }
    //emit displayRobots();
}

void RobotsController::shortcutDeleteRobot(){
    if(robots.size() > 0){
        robotIsDeadSlot(QString::number(robots.size() - 1));
        //emit displayRobots();
    } else
        qDebug() << "You already have no robot";
}

void RobotsController::sendCommand(QString ip, QString cmd){
    robots.value(ip)->sendCommand(cmd);
}

void RobotsController::newRobotPosSlot(QString ip, float posX, float posY, float ori){
    emit newRobotPos(ip, posX, posY, ori);
}

void RobotsController::setRobotPos(QString ip, float posX, float posY, float ori){
    emit setPos(ip, posX, posY, ori);
}

void RobotsController::newMetadataSlot(int width, int height, float resolution, float originX, float originY){
    emit newMetadata(width, height, resolution, originX, originY);
}

void RobotsController::updatePathSlot(QString ip, QStringList strList){
    emit updatePath(ip, strList);
}

void RobotsController::updateHomeSlot(QString ip, QString homeName, float homeX, float homeY){
    emit updateHome(ip, homeName, homeX, homeY);
}

void RobotsController::sendCommandNewName(QString ip, QString name){
    sendCommand(ip, QString("a") + QChar(31) + name);
}

void RobotsController::updateNameSlot(QString ip, QString name){
    emit setName(ip, name);
}

void RobotsController::sendCommandDeletePath(QString ip){
    sendCommand(ip, QString("m"));
}

void RobotsController::stoppedDeletedPathSlot(QString ip){
    emit setPath(ip, "");
    emit setPlayingPath(ip, false);
}
